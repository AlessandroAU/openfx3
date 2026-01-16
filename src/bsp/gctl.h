/*
 * Copyright (C) 2018 Marcus Comstedt
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef BSP_GCTL_H_
#define BSP_GCTL_H_

#include <stdint.h>

/* Crystal frequency */
#define XTAL_FREQ 19200000

/* Hard limit for the internal GPIF/bus clock. Can be overridden at build time.
 * Default is 100 MHz. Closest achievable is 99.84 MHz (FBDIV=26, div=9).
 */
#ifndef FX3_BUS_CLK_MAX_HZ
#define FX3_BUS_CLK_MAX_HZ 100000000UL
#endif

/* Runtime PLL reprogramming is always enabled. */
#define FX3_ALLOW_PLL_REPROGRAM 1

/* PLL feedback divider options (with 19.2 MHz crystal):
 * Note: only the first two of these values are officially supported
 * the others are overclocked settings, but they appear to work fine.
 * 20 -> SYS_CLK = 384.0 MHz
 * 21 -> SYS_CLK = 403.2 MHz
 * 22 -> SYS_CLK = 422.4 MHz
 * 23 -> SYS_CLK = 441.6 MHz
 * 24 -> SYS_CLK = 460.8 MHz
 * 25 -> SYS_CLK = 480.0 MHz
 * 26 -> SYS_CLK = 499.2 MHz
 */
#define PLL_FBDIV_MIN 20
#define PLL_FBDIV_MAX 27

/* Default PLL feedback divider (can be overridden via -DPLL_FBDIV=xx) */
#ifndef PLL_FBDIV
#define PLL_FBDIV 20
#endif

/* Compile-time clock values (for peripheral init calculations) */
#define SYS_CLK   (XTAL_FREQ * PLL_FBDIV)

#ifndef CPU_DIV
#define CPU_DIV 2
#endif

#define CPU_CLK   (SYS_CLK / CPU_DIV)

/* Clock configuration result */
typedef struct {
    uint8_t  pll_fbdiv;       /* PLL feedback divider (20-26) */
    uint16_t gpif_div;        /* GPIF clock divider register value */
    uint32_t sys_clk_hz;      /* Actual system clock in Hz */
    uint32_t bus_clk_hz;      /* Actual bus/GPIF clock in Hz */
} Fx3ClockConfig_t;

/* Global current PLL setting (updated by Fx3GctlSetBusClock) */
extern uint8_t Fx3GctlCurrentPllFbdiv;


/* Alternate function for GPIO 33-57 */
typedef enum {
  FX3_GCTL_ALTFUNC_GPIO = 0,
  FX3_GCTL_ALTFUNC_GPIO_UART = 1,
  FX3_GCTL_ALTFUNC_GPIO_SPI = 2,
  FX3_GCTL_ALTFUNC_GPIO_I2S = 3,
  FX3_GCTL_ALTFUNC_UART_SPI_I2S = 4,
  FX3_GCTL_ALTFUNC_GPIF32BIT_UART_I2S = 5
} Fx3GctlPinAltFunc_t;

extern void Fx3GctlInitClock(void);
extern void Fx3GctlInitIoMatrix(Fx3GctlPinAltFunc_t alt_func);
extern void Fx3GctlHardReset(void);

/* Read the PLL reference clock selection (FSLC field in PLL_CFG). */
extern uint32_t Fx3GctlGetPllFslc(void);

/* Return the PLL reference clock in Hz derived from FSLC. */
extern uint32_t Fx3GctlGetPllRefClkHz(void);

/* Calculate best clock configuration for requested bus speed.
 * target_bus_mhz: Desired bus/GPIF clock in MHz
 * config: Output structure with selected PLL and divider settings
 * Returns: Actual bus clock frequency in Hz */
extern uint32_t Fx3GctlCalcBusClock(uint32_t target_bus_mhz, Fx3ClockConfig_t *config);

/* Set system clock to achieve requested bus speed.
 * target_bus_mhz: Desired bus/GPIF clock in MHz
 * config: Output structure with actual settings used (can be NULL)
 * Returns: Actual bus clock frequency in Hz */
extern uint32_t Fx3GctlSetBusClock(uint32_t target_bus_mhz, Fx3ClockConfig_t *config);

/* Get current system clock frequency in Hz */
extern uint32_t Fx3GctlGetSysClk(void);

#endif /* BSP_GCTL_H_ */
