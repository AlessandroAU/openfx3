#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <stdint.h>

extern void start_benchmark(void);
extern void stop_benchmark(void);
extern uint8_t is_benchmark_active(void);

#endif /* BENCHMARK_H */
