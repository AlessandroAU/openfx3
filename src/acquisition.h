#ifndef ACQUISITION_H
#define ACQUISITION_H

#include <stdint.h>
#include "command.h"

extern void start_acquisition(uint16_t clock_divisor_x2, const struct acq_config *config);
extern void start_benchmark(void);
extern void stop_acquisition(void);
extern void poll_acquisition(void);

extern void poll_acquisition_debug(void);

/* Stats counters - incremented by ISR */
extern volatile uint64_t acq_total_buffers;  /* Buffers successfully committed */
extern volatile uint32_t acq_stalls;         /* Backpressure events (flow control) */

/* Current bus configuration (set by start_acquisition) */
extern uint32_t acq_bus_freq_hz;  /* Actual bus clock frequency in Hz */
extern uint32_t acq_sys_clk_hz;   /* System clock frequency in Hz */
extern uint16_t acq_gpif_div;     /* GPIF divider register value */
extern uint8_t acq_pll_fbdiv;     /* PLL feedback divider (20-27) */
extern uint8_t acq_bus_width;     /* Bus width in bits */

extern void reset_drop_stats(void);         /* Reset all stats counters */
extern uint64_t get_acq_total_bytes(void);  /* Total bytes committed */
extern uint8_t is_acquisition_active(void); /* Returns 1 if GPIF is running */

#endif /* ACQUISITION_H */
