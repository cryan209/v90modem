/* async_capture.h -- bounded diagnostic-only byte capture.
 * Not an audio buffer: failure never changes live sample accounting or asks
 * the engine to wait for diagnostic storage.
 * One serialized producer per handle; one private disk-writer thread.
 * open/flush/close are control-thread operations. Stop producers before close.
 * On overflow or I/O failure the capture fails closed: retain only a prefix,
 * reject all later bytes, and mark <path>.status.json incomplete. Never splice
 * later samples onto the prefix. Raw data is usable as a complete capture only
 * after clean close and a status file with state=complete and error=0.
 */
#ifndef ASYNC_CAPTURE_H
#define ASYNC_CAPTURE_H

#include <stddef.h>
#include <stdint.h>

#define ASYNC_CAPTURE_DEFAULT_CAPACITY (256 * 1024)

typedef struct async_capture async_capture_t;
typedef struct {
    uint64_t offered_bytes, accepted_bytes, written_bytes, rejected_bytes;
    int error;
} async_capture_stats_t;

/* Truncates the data and status files. Fails rather than enabling the sink if
 * metadata cannot be created or the required atomics are not lock-free. */
async_capture_t *async_capture_open(const char *path, size_t capacity);
/* No allocation, lock acquisition, filesystem calls, logging or waiting.
 * Returns 1 if this entire block was queued, 0 if rejected. No partial blocks. */
int async_capture_push(async_capture_t *s, const void *bytes, size_t len);
/* Wait for the accepted prefix at entry to reach write(), not fsync().
 * Returns -1 on capture failure (errno set); never call on the media clock. */
int async_capture_flush(async_capture_t *s);
/* Drain, join, finalize metadata and free. Returns -1 on capture failure.
 * Final counters are optional. A slow filesystem can delay control teardown,
 * but cannot block the producer or grow the queue. */
int async_capture_close(async_capture_t *s, async_capture_stats_t *stats);

#ifdef ASYNC_CAPTURE_TEST
#include <sys/types.h>
/* Deterministic stall/short-write/error injection, not in the production ABI. */
typedef ssize_t (*async_capture_test_write_t)(int, const void *, size_t, void *);
async_capture_t *async_capture_open_test(const char *path, size_t capacity,
                                         async_capture_test_write_t write_fn,
                                         void *user);
#endif
#endif
