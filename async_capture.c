/* async_capture.c -- disk I/O isolation for optional diagnostic taps.
 * See async_capture.h for the SPSC/lifecycle contract and fail-stop policy.
 * The producer publishes complete blocks with release/acquire ordering.
 * Tail advances only after write() returns, so a blocked disk cannot expose
 * queue storage for reuse. Only the diagnostic copy is buffered: live G.711
 * codewords never pass through this queue or wait for its disk writer.
 */
#define _POSIX_C_SOURCE 200809L
#include "async_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>

struct async_capture {
    unsigned char *queue;
    char *path;
    size_t capacity;
    int fd, status_fd;
    pthread_t thread;
    atomic_uint_fast64_t head, tail, offered, rejected;
    atomic_int error;
    atomic_bool stop, done, io_failed;
#ifdef ASYNC_CAPTURE_TEST
    async_capture_test_write_t write_fn;
    void *write_user;
#endif
};

static void fail(async_capture_t *s, int error)
{
    int expected = 0;
    atomic_compare_exchange_strong_explicit(&s->error, &expected, error,
                                            memory_order_relaxed, memory_order_relaxed);
}

static void snapshot(async_capture_t *s, async_capture_stats_t *stats)
{
    stats->offered_bytes = atomic_load_explicit(&s->offered, memory_order_relaxed);
    stats->accepted_bytes = atomic_load_explicit(&s->head, memory_order_acquire);
    stats->written_bytes = atomic_load_explicit(&s->tail, memory_order_acquire);
    stats->rejected_bytes = atomic_load_explicit(&s->rejected, memory_order_relaxed);
    stats->error = atomic_load_explicit(&s->error, memory_order_relaxed);
}

/* Only the control thread before start, then only the worker, writes metadata.
 * An interrupted/failed status update is not a valid completion certificate. */
static int write_status(async_capture_t *s, const char *state)
{
    char buf[512];
    async_capture_stats_t stats;
    snapshot(s, &stats);
    int n = snprintf(buf, sizeof(buf),
        "{\"version\":1,\"state\":\"%s\",\"prefix_only\":true,"
        "\"capacity_bytes\":%zu,\"offered_bytes\":%llu,\"accepted_bytes\":%llu,"
        "\"written_bytes\":%llu,\"rejected_bytes\":%llu,\"error\":%d}\n",
        state, s->capacity, (unsigned long long)stats.offered_bytes,
        (unsigned long long)stats.accepted_bytes, (unsigned long long)stats.written_bytes,
        (unsigned long long)stats.rejected_bytes, stats.error);
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        errno = EOVERFLOW;
        return -1;
    }
    size_t pos = 0;
    while (pos < (size_t)n) {
        ssize_t wrote = pwrite(s->status_fd, buf + pos, (size_t)n - pos, (off_t)pos);
        if (wrote < 0 && errno == EINTR)
            continue;
        if (wrote <= 0) {
            if (!wrote) errno = EIO;
            return -1;
        }
        pos += (size_t)wrote;
    }
    return ftruncate(s->status_fd, n);
}

static void pause_writer(void)
{
    /* Only enabled captures have a worker. No producer-side condvar/mutex or
     * wake syscall: at 8 kB/s, 10 ms polling leaves ample 256 KiB headroom. */
    const struct timespec delay = {0, 10000000};
    nanosleep(&delay, NULL);
}

static void *writer(void *arg)
{
    async_capture_t *s = arg;
    bool reported = false;
    bool io_failed = false;
    for (;;) {
        uint64_t tail = atomic_load_explicit(&s->tail, memory_order_relaxed);
        uint64_t head = atomic_load_explicit(&s->head, memory_order_acquire);
        if (tail != head && !io_failed) {
            size_t offset = (size_t)(tail % s->capacity);
            size_t n = s->capacity - offset;
            if ((uint64_t)n > head - tail)
                n = (size_t)(head - tail);
            ssize_t wrote;
#ifdef ASYNC_CAPTURE_TEST
            if (s->write_fn)
                wrote = s->write_fn(s->fd, s->queue + offset, n, s->write_user);
            else
#endif
                wrote = write(s->fd, s->queue + offset, n);
            if (wrote < 0 && errno == EINTR)
                continue;
            if (wrote <= 0) {
                fail(s, wrote < 0 ? errno : EIO);
                io_failed = true;
                atomic_store_explicit(&s->io_failed, true, memory_order_release);
            } else {
                atomic_store_explicit(&s->tail, tail + (uint64_t)wrote, memory_order_release);
            }
        }
        if (atomic_load_explicit(&s->error, memory_order_relaxed) && !reported) {
            if (write_status(s, "incomplete") < 0)
                fail(s, errno);
            fprintf(stderr, "[CAPTURE] %s incomplete: %s; retaining a prefix only\n",
                    s->path, strerror(atomic_load_explicit(&s->error, memory_order_relaxed)));
            reported = true;
        }
        if (atomic_load_explicit(&s->stop, memory_order_acquire)
            && (io_failed || atomic_load_explicit(&s->tail, memory_order_acquire)
                            == atomic_load_explicit(&s->head, memory_order_acquire)))
            break;
        if (tail == head || io_failed)
            pause_writer();
    }
    if (close(s->fd) < 0)
        fail(s, errno);
    const char *state = atomic_load_explicit(&s->error, memory_order_relaxed)
                      ? "incomplete" : "complete";
    if (write_status(s, state) < 0)
        fail(s, errno);
    if (close(s->status_fd) < 0)
        fail(s, errno);
    atomic_store_explicit(&s->done, true, memory_order_release);
    return NULL;
}

static int open_regular(const char *path)
{
    /* Reject pipes/devices: a diagnostic FIFO must not block setup or send
     * SIGPIPE into the modem. O_NONBLOCK has no effect on regular file I/O. */
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NONBLOCK, 0666);
    if (fd < 0)
        return -1;
    struct stat st;
    int result = fstat(fd, &st);
    if (result < 0 || !S_ISREG(st.st_mode)) {
        int saved = result < 0 ? errno : EINVAL;
        close(fd);
        errno = saved;
        return -1;
    }
    return fd;
}

static async_capture_t *open_capture(const char *path, size_t capacity
#ifdef ASYNC_CAPTURE_TEST
                                     , async_capture_test_write_t write_fn, void *user
#endif
                                     )
{
    if (!path || !*path || !capacity || capacity > UINT64_MAX / 2) {
        errno = EINVAL;
        return NULL;
    }
    async_capture_t *s = calloc(1, sizeof(*s));
    if (!s)
        return NULL;
    s->fd = s->status_fd = -1;
    s->capacity = capacity;
    atomic_init(&s->head, 0);
    atomic_init(&s->tail, 0);
    atomic_init(&s->offered, 0);
    atomic_init(&s->rejected, 0);
    atomic_init(&s->error, 0);
    atomic_init(&s->stop, false);
    atomic_init(&s->done, false);
    atomic_init(&s->io_failed, false);
    if (!atomic_is_lock_free(&s->head) || !atomic_is_lock_free(&s->tail)
        || !atomic_is_lock_free(&s->offered) || !atomic_is_lock_free(&s->rejected)
        || !atomic_is_lock_free(&s->error) || !atomic_is_lock_free(&s->stop)
        || !atomic_is_lock_free(&s->done) || !atomic_is_lock_free(&s->io_failed)) {
        errno = ENOTSUP;
        goto failed;
    }
    s->path = strdup(path);
    s->queue = malloc(capacity);
    if (!s->path || !s->queue)
        goto failed;
    /* Fault in the diagnostic storage during setup, not on its first media
     * enqueue. This is not mlock: OS scheduling/paging still applies. */
    long page = sysconf(_SC_PAGESIZE);
    size_t stride = page > 0 ? (size_t)page : 4096;
    volatile unsigned char *storage = s->queue;
    for (size_t pos = 0; pos < capacity; pos += stride)
        storage[pos] = 0;
    storage[capacity - 1] = 0;
    size_t path_len = strlen(path);
    if (path_len > SIZE_MAX - sizeof(".status.json")) {
        errno = EOVERFLOW;
        goto failed;
    }
    char *status_path = malloc(path_len + sizeof(".status.json"));
    if (!status_path)
        goto failed;
    memcpy(status_path, path, path_len);
    memcpy(status_path + path_len, ".status.json", sizeof(".status.json"));
    /* Invalidate any old completion certificate BEFORE truncating raw data. */
    s->status_fd = open_regular(status_path);
    free(status_path);
    if (s->status_fd < 0 || write_status(s, "recording") < 0)
        goto failed;
    s->fd = open_regular(path);
    if (s->fd < 0)
        goto failed;
#ifdef ASYNC_CAPTURE_TEST
    s->write_fn = write_fn;
    s->write_user = user;
#endif
    int err = pthread_create(&s->thread, NULL, writer, s);
    if (err) {
        errno = err;
        goto failed;
    }
    return s;
failed:;
    int saved = errno;
    if (s->fd >= 0) close(s->fd);
    if (s->status_fd >= 0) close(s->status_fd);
    free(s->queue);
    free(s->path);
    free(s);
    errno = saved;
    return NULL;
}

async_capture_t *async_capture_open(const char *path, size_t capacity)
{
    return open_capture(path, capacity
#ifdef ASYNC_CAPTURE_TEST
                        , NULL, NULL
#endif
                        );
}

#ifdef ASYNC_CAPTURE_TEST
async_capture_t *async_capture_open_test(const char *path, size_t capacity,
                                         async_capture_test_write_t write_fn, void *user)
{
    return open_capture(path, capacity, write_fn, user);
}
#endif

int async_capture_push(async_capture_t *s, const void *bytes, size_t len)
{
    if (!len)
        return 1;
    if (!s)
        return 0;
    atomic_fetch_add_explicit(&s->offered, len, memory_order_relaxed);
    uint64_t head = atomic_load_explicit(&s->head, memory_order_relaxed);
    uint64_t tail = atomic_load_explicit(&s->tail, memory_order_acquire);
    if (!bytes || len > s->capacity - (size_t)(head - tail))
        fail(s, bytes ? ENOBUFS : EINVAL);
    if (atomic_load_explicit(&s->error, memory_order_relaxed)) {
        atomic_fetch_add_explicit(&s->rejected, len, memory_order_relaxed);
        return 0;
    }
    size_t offset = (size_t)(head % s->capacity);
    size_t first = len < s->capacity - offset ? len : s->capacity - offset;
    memcpy(s->queue + offset, bytes, first);
    if (len > first)
        memcpy(s->queue, (const unsigned char *)bytes + first, len - first);
    atomic_store_explicit(&s->head, head + len, memory_order_release);
    return 1;
}

int async_capture_flush(async_capture_t *s)
{
    if (!s)
        return 0;
    uint64_t target = atomic_load_explicit(&s->head, memory_order_acquire);
    while (atomic_load_explicit(&s->tail, memory_order_acquire) < target
           && !atomic_load_explicit(&s->done, memory_order_acquire)
           && !atomic_load_explicit(&s->io_failed, memory_order_acquire))
        pause_writer();
    int error = atomic_load_explicit(&s->error, memory_order_relaxed);
    if (error) {
        errno = error;
        return -1;
    }
    return 0;
}

int async_capture_close(async_capture_t *s, async_capture_stats_t *stats)
{
    if (!s)
        return 0;
    atomic_store_explicit(&s->stop, true, memory_order_release);
    int error = pthread_join(s->thread, NULL);
    if (error) {
        /* A valid handle has exactly one control-thread owner. Do not free
         * storage if a violated lifecycle contract leaves the worker alive. */
        errno = error;
        return -1;
    }
    async_capture_stats_t final;
    snapshot(s, &final);
    if (stats)
        *stats = final;
    free(s->queue);
    free(s->path);
    free(s);
    if (final.error) {
        errno = final.error;
        return -1;
    }
    return 0;
}
