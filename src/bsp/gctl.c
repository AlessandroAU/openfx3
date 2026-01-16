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

#include <bsp/gctl.h>
#include <bsp/uart.h>
#include <bsp/regaccess.h>
#include <bsp/util.h>
#include <rdb/gctl.h>
#include <stdio.h>

/* Current PLL feedback divider - defaults to compile-time PLL_FBDIV */
uint8_t Fx3GctlCurrentPllFbdiv = PLL_FBDIV;

static uint32_t Fx3GctlOutdivDivisor(uint32_t outdiv)
{
  /* OUTDIV encoding is assumed to be power-of-two divisors (common PLL encoding). */
  switch (outdiv & 0x03) {
  case 0:
    return 1;
  case 1:
    return 2;
  case 2:
    return 4;
  default:
    return 8;
  }
}

static uint32_t Fx3GctlGetSysClkForFbdiv(uint8_t fbdiv)
{
  uint32_t pll_cfg = Fx3ReadReg32(FX3_GCTL_PLL_CFG);
  uint32_t ref_hz = Fx3GctlGetPllRefClkHz();

  /* REFDIV: /2 when set (e.g. 38.4 MHz -> effective 19.2 MHz). */
  if (pll_cfg & FX3_GCTL_PLL_CFG_REFDIV)
    ref_hz /= 2;

  /* OUTDIV: divide SYSCLK by encoded divisor. */
  uint32_t outdiv = (pll_cfg & FX3_GCTL_PLL_CFG_OUTDIV_MASK) >> FX3_GCTL_PLL_CFG_OUTDIV_SHIFT;
  uint32_t div = Fx3GctlOutdivDivisor(outdiv);
  if (div == 0)
    div = 1;

  return (ref_hz * (uint32_t)fbdiv) / div;
}

static uint8_t Fx3GctlSelectChargePumpCfg(uint32_t sys_clk_hz)
{
  /* Empirical table based on SYSCLK/VCO range. Higher frequencies need higher CP current.
   * 0: 2.5uA, 1: 5uA, 2: 7.5uA, 3: 10uA
   */
  if (sys_clk_hz <= 403200000UL)
    return 0;
  if (sys_clk_hz <= 460800000UL)
    return 1;
  if (sys_clk_hz <= 480000000UL)
    return 2;
  return 3;
}

uint32_t Fx3GctlGetPllFslc(void)
{
  return Fx3GetField32(FX3_GCTL_PLL_CFG, FSLC);
}

uint32_t Fx3GctlGetPllRefClkHz(void)
{
  /* FSLC[1:0] gives the reference frequency selection; FSLC[2] indicates source. */
  uint32_t fslc = Fx3GctlGetPllFslc();
  switch (fslc & 0x03) {
  case 0:
    return 19200000UL;
  case 1:
    return 26000000UL;
  case 2:
    return 38400000UL;
  default:
    return 52000000UL;
  }
}

static int Fx3GctlWaitForPllLock(uint32_t timeout_us)
{
  while (timeout_us--) {
    if ((Fx3ReadReg32(FX3_GCTL_PLL_CFG) & FX3_GCTL_PLL_CFG_PLL_LOCK) != 0)
      return 1;
    //Fx3UtilDelayUs(1);
  }
  return 0;
}

/* Internal: Set PLL to specified feedback divider */
static void Fx3GctlSetPll(uint8_t fbdiv)
{
  uint32_t current_pll_cfg = Fx3ReadReg32(FX3_GCTL_PLL_CFG);
  uint8_t current_fbdiv = (uint8_t)((current_pll_cfg & FX3_GCTL_PLL_CFG_FBDIV_MASK) >> FX3_GCTL_PLL_CFG_FBDIV_SHIFT);
  uint8_t current_cp = (uint8_t)((current_pll_cfg & FX3_GCTL_PLL_CFG_CP_CFG_MASK) >> FX3_GCTL_PLL_CFG_CP_CFG_SHIFT);

  uint32_t new_sys_clk = Fx3GctlGetSysClkForFbdiv(fbdiv);
  uint8_t new_cp = Fx3GctlSelectChargePumpCfg(new_sys_clk);

  if (current_fbdiv == fbdiv && current_cp == new_cp) {
    Fx3GctlCurrentPllFbdiv = current_fbdiv;
    return;
  }

  uint32_t new_pll_cfg = current_pll_cfg;
  new_pll_cfg &= ~(FX3_GCTL_PLL_CFG_FBDIV_MASK | FX3_GCTL_PLL_CFG_CP_CFG_MASK);
  new_pll_cfg |= (((uint32_t)fbdiv << FX3_GCTL_PLL_CFG_FBDIV_SHIFT) & FX3_GCTL_PLL_CFG_FBDIV_MASK);
  new_pll_cfg |= (((uint32_t)new_cp << FX3_GCTL_PLL_CFG_CP_CFG_SHIFT) & FX3_GCTL_PLL_CFG_CP_CFG_MASK);

  Fx3WriteReg32(FX3_GCTL_PLL_CFG, new_pll_cfg);
  (void)Fx3ReadReg32(FX3_GCTL_PLL_CFG);
  Fx3UtilDelayUs(10);

  /* Don't hang forever if PLL fails to lock. Revert to previous value if needed. */
  if (!Fx3GctlWaitForPllLock(20000)) {
    Fx3WriteReg32(FX3_GCTL_PLL_CFG, current_pll_cfg);
    (void)Fx3ReadReg32(FX3_GCTL_PLL_CFG);
    Fx3UtilDelayUs(10);
    (void)Fx3GctlWaitForPllLock(20000);
  }
  Fx3GctlCurrentPllFbdiv = Fx3GetField32(FX3_GCTL_PLL_CFG, FBDIV);
}

void Fx3GctlInitClock(void)
{
  /* Select minimum scalers for all clocks */
  Fx3WriteReg32(FX3_GCTL_CPU_CLK_CFG,
		(1UL << FX3_GCTL_CPU_CLK_CFG_MMIO_DIV_SHIFT) |
		(1UL << FX3_GCTL_CPU_CLK_CFG_DMA_DIV_SHIFT) |
		(3UL << FX3_GCTL_CPU_CLK_CFG_SRC_SHIFT) |
		((CPU_DIV - 1UL) << FX3_GCTL_CPU_CLK_CFG_CPU_DIV_SHIFT));
  Fx3UtilDelayUs(10);

  /* Set PLL to compile-time default */
  Fx3GctlSetPll(PLL_FBDIV);
}

uint32_t Fx3GctlGetSysClk(void)
{
  return Fx3GctlGetSysClkForFbdiv(Fx3GctlCurrentPllFbdiv);
}

uint32_t Fx3GctlCalcBusClock(uint32_t target_bus_mhz, Fx3ClockConfig_t *config)
{
  /* Enforce reasonable limits: 1 MHz minimum (lower frequencies have timing issues),
   * and cap at FX3_BUS_CLK_MAX_HZ (default 100 MHz). */
  if (target_bus_mhz < 1)
    target_bus_mhz = 1;

  uint32_t target_hz = target_bus_mhz * 1000000UL;
  const uint32_t absolute_max_hz = FX3_BUS_CLK_MAX_HZ;
  if (target_hz > absolute_max_hz)
    target_hz = absolute_max_hz;

  uint32_t best_error = 0xFFFFFFFF;
  uint8_t best_fbdiv = PLL_FBDIV_MIN;
  uint16_t best_div = 3;  /* Must be odd to avoid HALFDIV */
  uint32_t best_bus_hz = 0;

  /* Search all PLL settings from HIGH to LOW
   * Lower PLL frequencies reduce power consumption and may work better at very low
   * bus frequencies where the sys_clk/bus_clk ratio would otherwise be extreme.
   */
  for (uint8_t fbdiv = PLL_FBDIV_MAX; fbdiv >= PLL_FBDIV_MIN; fbdiv--) {
    uint32_t sys_clk = Fx3GctlGetSysClkForFbdiv(fbdiv);

    /* GPIF divider is half-integer: actual_div = (div_reg + 1) / 2
     * div_reg=2 -> 1.5, div_reg=3 -> 2.0, div_reg=4 -> 2.5, etc.
     * DIV register is 10 bits (max 1023), allowing sub-MHz frequencies.
     *
     * Only use ODD div_reg values to avoid HALFDIV mode which causes
     * data corruption at thread boundaries. Odd div_reg -> even clock_divisor_x2.
     */
    uint16_t max_div = 1023;

    for (uint16_t div_reg = 3; div_reg <= max_div; div_reg += 2) {
      /* bus_clk = sys_clk * 2 / (div_reg + 1) */
      uint32_t bus_hz = (sys_clk * 2) / (div_reg + 1);

      /* Never exceed 100 MHz absolute limit */
      if (bus_hz > absolute_max_hz)
        continue;

      /* Find closest match (can be over or under target, just not over 100 MHz) */
      uint32_t error = (bus_hz > target_hz) ? (bus_hz - target_hz) : (target_hz - bus_hz);
      if (error < best_error) {
        best_error = error;
        best_fbdiv = fbdiv;
        best_div = div_reg;
        best_bus_hz = bus_hz;
      }

      /* Exact match - stop early */
      if (error == 0)
        break;
    }
  }


  if (config) {
    config->pll_fbdiv = best_fbdiv;
    config->gpif_div = best_div;
    config->sys_clk_hz = Fx3GctlGetSysClkForFbdiv(best_fbdiv);
    config->bus_clk_hz = best_bus_hz;
  }

  return best_bus_hz;
}

uint32_t Fx3GctlSetBusClock(uint32_t target_bus_mhz, Fx3ClockConfig_t *config)
{
  Fx3ClockConfig_t cfg;
  char buf[96];

  /* Calculate best configuration */
  Fx3GctlCalcBusClock(target_bus_mhz, &cfg);

  /* Apply PLL setting */
  Fx3GctlSetPll(cfg.pll_fbdiv);

  /* Update UART baud rate for new PLL setting */
  Fx3UartUpdateBaud(115200);

  /* Debug output */
  snprintf(buf, sizeof(buf), "Clock: target=%luMHz -> PLL=%u (sys=%luMHz) div=%u -> bus=%lu.%02luMHz\n",
           (unsigned long)target_bus_mhz,
           cfg.pll_fbdiv,
           (unsigned long)(cfg.sys_clk_hz / 1000000),
           cfg.gpif_div,
           (unsigned long)(cfg.bus_clk_hz / 1000000),
           (unsigned long)((cfg.bus_clk_hz % 1000000) / 10000));
  Fx3UartTxString(buf);

  /* Copy config if requested */
  if (config) {
    *config = cfg;
  }

  return cfg.bus_clk_hz;
}

void Fx3GctlInitIoMatrix(Fx3GctlPinAltFunc_t alt_func)
{
  /* Disable all GPIO overrides */
  Fx3WriteReg32(FX3_GCTL_GPIO_SIMPLE, 0);
  Fx3WriteReg32(FX3_GCTL_GPIO_SIMPLE+4, 0);
  Fx3WriteReg32(FX3_GCTL_GPIO_COMPLEX, 0);
  Fx3WriteReg32(FX3_GCTL_GPIO_COMPLEX+4, 0);

  Fx3UtilDelayUs(1);

  /* Configure matrix */
  Fx3WriteReg32(FX3_GCTL_IOMATRIX,
		(alt_func << FX3_GCTL_IOMATRIX_S1CFG_SHIFT) |
		(alt_func == FX3_GCTL_ALTFUNC_GPIF32BIT_UART_I2S?
		 FX3_GCTL_IOMATRIX_S0CFG : 0));
}

void Fx3GctlHardReset(void)
{
  Fx3UtilDelayUs(5);
  Fx3ClearReg32(FX3_GCTL_CONTROL, FX3_GCTL_CONTROL_HARD_RESET_N);
  Fx3UtilDelayUs(5);
}
