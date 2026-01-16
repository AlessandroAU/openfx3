/*
 * Platform abstraction layer for test utilities
 *
 * Provides cross-platform helpers for timing, sleeping, and other
 * OS-specific functionality.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#include <malloc.h>
#else
#include <time.h>
#include <unistd.h>
#endif

/*
 * Sleep for specified milliseconds
 */
static inline void sleep_ms(int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

/*
 * Sleep for specified microseconds
 */
static inline void sleep_us(int us) {
#ifdef _WIN32
    /* Windows doesn't have sub-millisecond sleep, use busy-wait for short durations */
    if (us < 1000) {
        LARGE_INTEGER freq, start, now;
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&start);
        double target = (double)us / 1000000.0;
        do {
            QueryPerformanceCounter(&now);
        } while ((double)(now.QuadPart - start.QuadPart) / (double)freq.QuadPart < target);
    } else {
        Sleep(us / 1000);
    }
#else
    usleep(us);
#endif
}

/*
 * High-resolution timer for measuring elapsed time
 */
typedef struct {
#ifdef _WIN32
    LARGE_INTEGER freq;
    LARGE_INTEGER start;
#else
    struct timespec start;
#endif
} platform_timer_t;

static inline void platform_timer_init(platform_timer_t *t) {
#ifdef _WIN32
    QueryPerformanceFrequency(&t->freq);
#else
    (void)t;
#endif
}

static inline void platform_timer_start(platform_timer_t *t) {
#ifdef _WIN32
    QueryPerformanceCounter(&t->start);
#else
    clock_gettime(CLOCK_MONOTONIC, &t->start);
#endif
}

static inline double platform_timer_elapsed_us(platform_timer_t *t) {
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (double)(now.QuadPart - t->start.QuadPart) * 1000000.0 / (double)t->freq.QuadPart;
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->start.tv_sec) * 1000000.0 +
           (now.tv_nsec - t->start.tv_nsec) / 1000.0;
#endif
}

static inline double platform_timer_elapsed_ms(platform_timer_t *t) {
    return platform_timer_elapsed_us(t) / 1000.0;
}

static inline double platform_timer_elapsed_sec(platform_timer_t *t) {
    return platform_timer_elapsed_us(t) / 1000000.0;
}

/*
 * Interval timer - tracks both total elapsed time and interval time
 * Useful for periodic stats updates while tracking overall duration
 */
typedef struct {
#ifdef _WIN32
    LARGE_INTEGER freq;
    LARGE_INTEGER start;
    LARGE_INTEGER interval_start;
#else
    struct timespec start;
    struct timespec interval_start;
#endif
} platform_interval_timer_t;

static inline void platform_interval_timer_init(platform_interval_timer_t *t) {
#ifdef _WIN32
    QueryPerformanceFrequency(&t->freq);
#else
    (void)t;
#endif
}

static inline void platform_interval_timer_start(platform_interval_timer_t *t) {
#ifdef _WIN32
    QueryPerformanceCounter(&t->start);
    t->interval_start = t->start;
#else
    clock_gettime(CLOCK_MONOTONIC, &t->start);
    t->interval_start = t->start;
#endif
}

static inline double platform_interval_timer_elapsed_sec(platform_interval_timer_t *t) {
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (double)(now.QuadPart - t->start.QuadPart) / (double)t->freq.QuadPart;
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->start.tv_sec) +
           (now.tv_nsec - t->start.tv_nsec) / 1e9;
#endif
}

static inline double platform_interval_timer_interval_ms(platform_interval_timer_t *t) {
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (double)(now.QuadPart - t->interval_start.QuadPart) * 1000.0 / (double)t->freq.QuadPart;
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->interval_start.tv_sec) * 1000.0 +
           (now.tv_nsec - t->interval_start.tv_nsec) / 1e6;
#endif
}

static inline void platform_interval_timer_reset_interval(platform_interval_timer_t *t) {
#ifdef _WIN32
    QueryPerformanceCounter(&t->interval_start);
#else
    clock_gettime(CLOCK_MONOTONIC, &t->interval_start);
#endif
}

/*
 * Aligned memory allocation (page-aligned for DMA buffers)
 */

static inline void *platform_aligned_alloc(size_t size, size_t alignment) {
#ifdef _WIN32
    return _aligned_malloc(size, alignment);
#else
    void *p = NULL;
    if (posix_memalign(&p, alignment, size) != 0) return NULL;
    return p;
#endif
}

static inline void platform_aligned_free(void *p) {
#ifdef _WIN32
    _aligned_free(p);
#else
    free(p);
#endif
}

/*
 * Thread support
 */

#ifdef _WIN32

/* Windows thread wrapper to bridge POSIX-style void*(*)(void*) to DWORD(*)(void*) */
typedef void *(*platform_thread_func_t)(void *);

typedef struct {
    platform_thread_func_t func;
    void *arg;
} platform_thread_wrapper_t;

static DWORD WINAPI platform_thread_wrapper(LPVOID param) {
    platform_thread_wrapper_t *wrapper = (platform_thread_wrapper_t *)param;
    platform_thread_func_t func = wrapper->func;
    void *arg = wrapper->arg;
    free(wrapper);
    func(arg);
    return 0;
}

typedef HANDLE platform_thread_t;

static inline int platform_thread_create(platform_thread_t *thread, void *(*func)(void *), void *arg) {
    platform_thread_wrapper_t *wrapper = (platform_thread_wrapper_t *)malloc(sizeof(*wrapper));
    if (!wrapper) return -1;
    wrapper->func = func;
    wrapper->arg = arg;
    *thread = CreateThread(NULL, 0, platform_thread_wrapper, wrapper, 0, NULL);
    if (*thread == NULL) {
        free(wrapper);
        return -1;
    }
    return 0;
}

static inline int platform_thread_join(platform_thread_t thread) {
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
    return 0;
}

#else
#include <pthread.h>
typedef pthread_t platform_thread_t;

static inline int platform_thread_create(platform_thread_t *thread, void *(*func)(void *), void *arg) {
    return pthread_create(thread, NULL, func, arg);
}

static inline int platform_thread_join(platform_thread_t thread) {
    return pthread_join(thread, NULL);
}

#endif

#endif /* PLATFORM_H */
