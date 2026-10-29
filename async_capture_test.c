/* Deterministic diagnostic queue tests; never need a slow real disk or modem. */
#define _POSIX_C_SOURCE 200809L
#define ASYNC_CAPTURE_TEST
#include "async_capture.h"
#include "test_tmp.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static void status_path(char *dst, size_t size, const char *path)
{
    assert(snprintf(dst, size, "%s.status.json", path) < (int)size);
}

static void cleanup(const char *path)
{
    char sidecar[1024];
    status_path(sidecar, sizeof(sidecar), path);
    unlink(path);
    unlink(sidecar);
}

static void check_file(const char *path, const void *want, size_t n)
{
    unsigned char bytes[4096];
    assert(n <= sizeof(bytes));
    FILE *f = fopen(path, "rb");
    assert(f);
    assert(fread(bytes, 1, sizeof(bytes), f) == n);
    assert(!ferror(f));
    assert(memcmp(bytes, want, n) == 0);
    fclose(f);
}

static void check_status(const char *path, const char *state,
                         uint64_t accepted, uint64_t written, uint64_t rejected, int error)
{
    char sidecar[1024], buf[1024], field[80];
    status_path(sidecar, sizeof(sidecar), path);
    FILE *f = fopen(sidecar, "r");
    assert(f);
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    assert(!ferror(f));
    buf[n] = '\0';
    fclose(f);
    snprintf(field, sizeof(field), "\"state\":\"%s\"", state);
    assert(strstr(buf, field));
    snprintf(field, sizeof(field), "\"accepted_bytes\":%llu,", (unsigned long long)accepted);
    assert(strstr(buf, field));
    snprintf(field, sizeof(field), "\"written_bytes\":%llu,", (unsigned long long)written);
    assert(strstr(buf, field));
    snprintf(field, sizeof(field), "\"rejected_bytes\":%llu,", (unsigned long long)rejected);
    assert(strstr(buf, field));
    snprintf(field, sizeof(field), "\"error\":%d}", error);
    assert(strstr(buf, field));
}

static void test_wrap_and_flush(void)
{
    const char *path = test_tmp("async-wrap.bin");
    async_capture_t *s = async_capture_open(path, 63);
    assert(s);
    check_status(path, "recording", 0, 0, 0, 0);
    unsigned char expected[1984];
    for (unsigned batch = 0; batch < 32; batch++) {
        for (unsigned j = 0; j < 62; j++)
            expected[batch * 62 + j] = (unsigned char)(batch * 17 + j);
        assert(async_capture_push(s, expected + batch * 62, 31));
        assert(async_capture_push(s, expected + batch * 62 + 31, 31));
        assert(async_capture_flush(s) == 0);
    }
    /* Flush publishes data without closing or certifying the capture. */
    check_file(path, expected, sizeof(expected));
    async_capture_stats_t stats;
    assert(async_capture_close(s, &stats) == 0);
    assert(stats.offered_bytes == sizeof(expected));
    assert(stats.accepted_bytes == sizeof(expected));
    assert(stats.written_bytes == sizeof(expected));
    assert(stats.rejected_bytes == 0 && stats.error == 0);
    check_status(path, "complete", sizeof(expected), sizeof(expected), 0, 0);
    cleanup(path);
}

typedef struct {
    enum { PAUSE, SHORT_EINTR, DISK_FULL } mode;
    atomic_bool entered, release;
    unsigned calls, written;
} hook_t;

static ssize_t hooked_write(int fd, const void *bytes, size_t n, void *user)
{
    hook_t *hook = user;
    if (hook->mode == PAUSE) {
        atomic_store_explicit(&hook->entered, true, memory_order_release);
        while (!atomic_load_explicit(&hook->release, memory_order_acquire)) {
            const struct timespec delay = {0, 1000000};
            nanosleep(&delay, NULL);
        }
    } else if (hook->mode == SHORT_EINTR) {
        if (!hook->calls++) {
            errno = EINTR;
            return -1;
        }
        if (n > 3) n = 3;
    } else {
        if (hook->written >= 5) {
            errno = ENOSPC;
            return -1;
        }
        if (n > 5 - hook->written) n = 5 - hook->written;
    }
    ssize_t wrote = write(fd, bytes, n);
    if (wrote > 0) hook->written += (unsigned)wrote;
    return wrote;
}

static void wait_entered(hook_t *hook)
{
    for (unsigned i = 0; i < 3000; i++) {
        if (atomic_load_explicit(&hook->entered, memory_order_acquire))
            return;
        const struct timespec delay = {0, 1000000};
        nanosleep(&delay, NULL);
    }
    assert(!"writer did not reach the injected stall");
}

static void test_blocked_disk_never_blocks_producer(void)
{
    const char *path = test_tmp("async-overflow.bin");
    hook_t hook = {.mode = PAUSE};
    atomic_init(&hook.entered, false);
    atomic_init(&hook.release, false);
    async_capture_t *s = async_capture_open_test(path, 32, hooked_write, &hook);
    assert(s);
    unsigned char expected[32];
    memset(expected, 'a', 16);
    memset(expected + 16, 'b', 16);
    assert(async_capture_push(s, expected, 16));
    wait_entered(&hook);
    /* Worker is held inside write(). Enqueue must still return immediately,
     * and must not reuse storage that the blocked write() is referencing. */
    assert(async_capture_push(s, expected + 16, 16));
    assert(!async_capture_push(s, "ccc", 3));
    assert(!async_capture_push(s, "dddd", 4));
    assert(!atomic_load_explicit(&hook.release, memory_order_relaxed));
    atomic_store_explicit(&hook.release, true, memory_order_release);
    assert(async_capture_flush(s) == -1 && errno == ENOBUFS);
    /* Even after space is freed, never resume/splice later samples. */
    assert(!async_capture_push(s, "e", 1));
    async_capture_stats_t stats;
    assert(async_capture_close(s, &stats) == -1 && errno == ENOBUFS);
    assert(stats.offered_bytes == 40 && stats.accepted_bytes == 32);
    assert(stats.written_bytes == 32 && stats.rejected_bytes == 8);
    check_file(path, expected, sizeof(expected));
    check_status(path, "incomplete", 32, 32, 8, ENOBUFS);
    cleanup(path);
}

static void test_short_writes_and_eintr(void)
{
    const char *path = test_tmp("async-short.bin");
    hook_t hook = {.mode = SHORT_EINTR};
    async_capture_t *s = async_capture_open_test(path, 64, hooked_write, &hook);
    assert(s);
    const char payload[] = "short writes preserve every octet";
    assert(async_capture_push(s, payload, sizeof(payload)));
    async_capture_stats_t stats;
    assert(async_capture_close(s, &stats) == 0);
    check_file(path, payload, sizeof(payload));
    check_status(path, "complete", sizeof(payload), sizeof(payload), 0, 0);
    cleanup(path);
}

static void test_disk_error_retains_only_written_prefix(void)
{
    const char *path = test_tmp("async-error.bin");
    hook_t hook = {.mode = DISK_FULL};
    async_capture_t *s = async_capture_open_test(path, 64, hooked_write, &hook);
    assert(s);
    assert(async_capture_push(s, "abcdefghijklmnop", 16));
    assert(async_capture_flush(s) == -1 && errno == ENOSPC);
    assert(!async_capture_push(s, "zzz", 3));
    async_capture_stats_t stats;
    assert(async_capture_close(s, &stats) == -1 && errno == ENOSPC);
    assert(stats.offered_bytes == 19 && stats.accepted_bytes == 16);
    assert(stats.written_bytes == 5 && stats.rejected_bytes == 3);
    check_file(path, "abcde", 5);
    check_status(path, "incomplete", 16, 5, 3, ENOSPC);
    cleanup(path);
}

static void test_oversized_block_is_never_partially_queued(void)
{
    const char *path = test_tmp("async-oversized.bin");
    async_capture_t *s = async_capture_open(path, 4);
    assert(s);
    assert(!async_capture_push(s, "abcde", 5));
    assert(!async_capture_push(s, "f", 1));
    async_capture_stats_t stats;
    assert(async_capture_close(s, &stats) == -1 && errno == ENOBUFS);
    assert(stats.offered_bytes == 6 && stats.rejected_bytes == 6);
    check_file(path, "", 0);
    check_status(path, "incomplete", 0, 0, 6, ENOBUFS);
    cleanup(path);
}

static void test_open_and_empty_lifecycle(void)
{
    const char *path = test_tmp("async-empty.bin");
    assert(!async_capture_open(path, 0) && errno == EINVAL);
    assert(!async_capture_open(NULL, 64) && errno == EINVAL);
    for (unsigned i = 0; i < 3; i++) {
        async_capture_t *s = async_capture_open(path, 64);
        assert(s);
        check_status(path, "recording", 0, 0, 0, 0);
        assert(async_capture_push(s, NULL, 0));
        if (i == 0)
            assert(async_capture_push(s, "old", 3));
        assert(async_capture_flush(s) == 0);
        assert(async_capture_close(s, NULL) == 0);
        unsigned n = i == 0 ? 3 : 0;
        check_status(path, "complete", n, n, 0, 0);
        check_file(path, "old", n);
    }
    cleanup(path);
    assert(async_capture_flush(NULL) == 0);
    assert(async_capture_close(NULL, NULL) == 0);
    assert(!async_capture_push(NULL, "x", 1));

    /* A missing/unwritable sidecar must not destroy an existing raw tap. */
    path = test_tmp("async-bad-sidecar.bin");
    char sidecar[1024];
    status_path(sidecar, sizeof(sidecar), path);
    FILE *f = fopen(path, "wb");
    assert(f && fwrite("old", 1, 3, f) == 3);
    fclose(f);
    assert(mkdir(sidecar, 0700) == 0);
    assert(!async_capture_open(path, 64));
    check_file(path, "old", 3);
    assert(rmdir(sidecar) == 0);
    cleanup(path);

    path = test_tmp("async-fifo");
    assert(mkfifo(path, 0600) == 0);
    assert(!async_capture_open(path, 64)); /* must not wait for a FIFO reader */
    cleanup(path);
}

int main(void)
{
    test_wrap_and_flush();
    test_blocked_disk_never_blocks_producer();
    test_short_writes_and_eintr();
    test_disk_error_retains_only_written_prefix();
    test_oversized_block_is_never_partially_queued();
    test_open_and_empty_lifecycle();
    puts("async_capture_test: PASS");
    return 0;
}
