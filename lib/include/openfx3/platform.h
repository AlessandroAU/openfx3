/*
 * Platform abstraction layer
 *
 * Provides cross-platform helpers for timing, sleeping, aligned allocation,
 * and threading. Supports Windows, Linux, and macOS.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#include <malloc.h>
#else
#include <errno.h>
#include <time.h>
#include <pthread.h>
#endif

/* ============================================================================
 * Sleep functions
 * ============================================================================ */

static inline void sleep_ms(unsigned int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
#endif
}

static inline void sleep_us(unsigned int us) {
#ifdef _WIN32
    if (us < 1000) {
        /* Windows Sleep() has ms resolution, busy-wait for short durations */
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
    struct timespec ts = { .tv_sec = us / 1000000, .tv_nsec = (us % 1000000) * 1000L };
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
#endif
}

/* ============================================================================
 * High-resolution timer
 * ============================================================================ */

typedef struct {
#ifdef _WIN32
    LARGE_INTEGER freq;
    LARGE_INTEGER start;
#else
    struct timespec start;
#endif
} platform_timer_t;

static inline void platform_timer_start(platform_timer_t *t) {
#ifdef _WIN32
    QueryPerformanceFrequency(&t->freq);
    QueryPerformanceCounter(&t->start);
#else
    clock_gettime(CLOCK_MONOTONIC, &t->start);
#endif
}

static inline double platform_timer_elapsed_us(const platform_timer_t *t) {
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

static inline double platform_timer_elapsed_ms(const platform_timer_t *t) {
    return platform_timer_elapsed_us(t) / 1000.0;
}

static inline double platform_timer_elapsed_sec(const platform_timer_t *t) {
    return platform_timer_elapsed_us(t) / 1000000.0;
}

/* ============================================================================
 * Interval timer - tracks total elapsed time and interval time
 * ============================================================================ */

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

static inline void platform_interval_timer_start(platform_interval_timer_t *t) {
#ifdef _WIN32
    QueryPerformanceFrequency(&t->freq);
    QueryPerformanceCounter(&t->start);
    t->interval_start = t->start;
#else
    clock_gettime(CLOCK_MONOTONIC, &t->start);
    t->interval_start = t->start;
#endif
}

static inline double platform_interval_timer_elapsed_sec(const platform_interval_timer_t *t) {
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

static inline double platform_interval_timer_interval_ms(const platform_interval_timer_t *t) {
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

/* ============================================================================
 * Aligned memory allocation
 * ============================================================================ */

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

/* ============================================================================
 * Threading
 * ============================================================================ */

#ifdef _WIN32

typedef HANDLE platform_thread_t;

typedef struct {
    void *(*func)(void *);
    void *arg;
} platform_thread_wrapper_t_;

static DWORD WINAPI platform_thread_wrapper_(LPVOID param) {
    platform_thread_wrapper_t_ *w = (platform_thread_wrapper_t_ *)param;
    void *(*func)(void *) = w->func;
    void *arg = w->arg;
    free(w);
    func(arg);
    return 0;
}

static inline int platform_thread_create(platform_thread_t *thread, void *(*func)(void *), void *arg) {
    platform_thread_wrapper_t_ *w = (platform_thread_wrapper_t_ *)malloc(sizeof(*w));
    if (!w) return -1;
    w->func = func;
    w->arg = arg;
    *thread = CreateThread(NULL, 0, platform_thread_wrapper_, w, 0, NULL);
    if (*thread == NULL) {
        free(w);
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

typedef pthread_t platform_thread_t;

static inline int platform_thread_create(platform_thread_t *thread, void *(*func)(void *), void *arg) {
    return pthread_create(thread, NULL, func, arg) == 0 ? 0 : -1;
}

static inline int platform_thread_join(platform_thread_t thread) {
    return pthread_join(thread, NULL);
}

#endif

#endif /* PLATFORM_H */
