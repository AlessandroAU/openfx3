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

#include <bsp/gpif.h>
#include <bsp/regaccess.h>
#include <bsp/irq.h>
#include <bsp/util.h>
#include <bsp/uart.h>
#include <rdb/gctl.h>
#include <rdb/gpif.h>
#include <rdb/pib.h>
#include <rdb/vic.h>
#include <stdio.h>

/* Uncomment to enable UART debug output (costs ~1KB flash) */
// #define DEBUG_UART

/* GPIF/PIB error counter - exported for stats reporting */
volatile uint32_t gpif_error_count = 0;

static void Fx3GpifPibIsr(void) __attribute__ ((isr ("IRQ"), optimize("O3")));

static void Fx3GpifPibIsr(void)
{
  /* Check for actual errors before clearing */
  uint32_t pib_intr = Fx3ReadReg32(FX3_PIB_INTR);
  uint32_t pib_error = Fx3ReadReg32(FX3_PIB_ERROR);

  /* Count GPIF or PIB errors */
  if (pib_intr & (FX3_PIB_INTR_GPIF_ERR | FX3_PIB_INTR_PIB_ERR)) {
    gpif_error_count++;
  }

  /* Clear all PIB/GPIF interrupts and errors */
  Fx3WriteReg32(FX3_PIB_INTR, pib_intr);
  Fx3WriteReg32(FX3_PIB_ERROR, pib_error);
  Fx3WriteReg32(FX3_GPIF_INTR, Fx3ReadReg32(FX3_GPIF_INTR));

  /* EOI */
  Fx3WriteReg32(FX3_VIC_ADDRESS, 0);
}


void Fx3GpifStart(uint8_t state, uint8_t alpha)
{
#ifdef DEBUG_UART
  {
    char _start_dbg[128];
    snprintf(_start_dbg, sizeof(_start_dbg), "Fx3GpifStart: state=%u alpha=%u\n", (unsigned)state, (unsigned)alpha);
    Fx3UartTxString(_start_dbg);
  }
#endif
  Fx3WriteReg32(FX3_GPIF_INTR, Fx3ReadReg32(FX3_GPIF_INTR));
  Fx3WriteReg32(FX3_GPIF_INTR_MASK,
		FX3_GPIF_INTR_MASK_GPIF_INTR | FX3_GPIF_INTR_MASK_GPIF_DONE);
  Fx3SetField32(FX3_GPIF_WAVEFORM_CTRL_STAT, ALPHA_INIT, alpha);
  Fx3SetReg32(FX3_GPIF_WAVEFORM_CTRL_STAT,
	      FX3_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID);
  Fx3WriteReg32(FX3_GPIF_WAVEFORM_SWITCH,
		(Fx3ReadReg32(FX3_GPIF_WAVEFORM_SWITCH) &
		 ~(FX3_GPIF_WAVEFORM_SWITCH_DESTINATION_STATE_SHIFT |
		   FX3_GPIF_WAVEFORM_SWITCH_TERMINAL_STATE_MASK |
		   FX3_GPIF_WAVEFORM_SWITCH_SWITCH_NOW |
		   FX3_GPIF_WAVEFORM_SWITCH_WAVEFORM_SWITCH)) |
		(state << FX3_GPIF_WAVEFORM_SWITCH_DESTINATION_STATE_SHIFT));
  Fx3SetReg32(FX3_GPIF_WAVEFORM_SWITCH,
	      FX3_GPIF_WAVEFORM_SWITCH_SWITCH_NOW |
	      FX3_GPIF_WAVEFORM_SWITCH_WAVEFORM_SWITCH);
#ifdef DEBUG_UART
  Fx3UartTxString("Fx3GpifStart: waveform switch requested\n");
#endif
}

void Fx3GpifStop(void)
{
  Fx3SetReg32(FX3_GPIF_WAVEFORM_CTRL_STAT, FX3_GPIF_WAVEFORM_CTRL_STAT_PAUSE);
  Fx3UtilDelayUs(10);
  Fx3WriteReg32(FX3_GPIF_WAVEFORM_CTRL_STAT, 0);
}

void Fx3GpifInvalidate(void)
{
  unsigned i;
  Fx3WriteReg32(FX3_GPIF_CONFIG, 0x220);
  for (i=0; i<256; i++) {
    Fx3WriteReg32(FX3_GPIF_LEFT_WAVEFORM + i*16 + 8, 0);
    Fx3WriteReg32(FX3_GPIF_RIGHT_WAVEFORM + i*16 + 8, 0);
  }
  for (i=0; i<4; i++)
    Fx3WriteReg32(FX3_GPIF_THREAD_CONFIG + i*4, 0);
}

static void Fx3GpifConfigureCommon(const uint16_t *functions,
				   uint16_t num_functions,
				   const uint32_t *registers,
				   uint16_t num_registers)
{
  unsigned i;
  if (functions && num_functions) {
    if (num_functions > 32)
      num_functions = 32;
    for(i=0; i<num_functions; i++) {
      Fx3WriteReg32(FX3_GPIF_FUNCTION + i*4, functions[i]);
    }
  }
  if (registers && num_registers) {
    for(i=num_registers; i--; )
      Fx3WriteReg32(FX3_GPIF_CONFIG + i*4, registers[i]);
  }
}


void Fx3GpifConfigure(const Fx3GpifWaveform_t *waveforms,
		      uint16_t num_waveforms,
		      const uint16_t *functions, uint16_t num_functions,
		      const Fx3GpifRegisters_t *registers)
{
  unsigned i;
  if (waveforms && num_waveforms) {
    for(i=0; i<num_waveforms; i++) {
      const Fx3GpifWaveform_t *left = &waveforms[waveforms[i].left];
      const Fx3GpifWaveform_t *right = &waveforms[waveforms[i].right];
      unsigned pos = waveforms[i].state[0] & FX3_GPIF_LEFT_WAVEFORM_NEXT_STATE_MASK;
      Fx3WriteReg32(FX3_GPIF_LEFT_WAVEFORM + pos*16 + 0, left->state[0]);
      Fx3WriteReg32(FX3_GPIF_LEFT_WAVEFORM + pos*16 + 4, left->state[1]);
      Fx3WriteReg32(FX3_GPIF_LEFT_WAVEFORM + pos*16 + 8, left->state[2]);
      Fx3WriteReg32(FX3_GPIF_RIGHT_WAVEFORM + pos*16 + 0, right->state[0]);
      Fx3WriteReg32(FX3_GPIF_RIGHT_WAVEFORM + pos*16 + 4, right->state[1]);
      Fx3WriteReg32(FX3_GPIF_RIGHT_WAVEFORM + pos*16 + 8, right->state[2]);
    }
  }
  Fx3GpifConfigureCommon(functions, num_functions, &registers->config,
			 sizeof(*registers)/sizeof(uint32_t));
}

void Fx3GpifConfigureCompat(const Fx3GpifWaveformCompat_t *waveforms,
			    const uint8_t *waveform_select,
			    uint16_t num_waveforms,
			    const uint16_t *functions, uint16_t num_functions,
			    const uint32_t *registers, uint16_t num_registers)
{
  unsigned i;
  if (waveforms && num_waveforms) {
    for(i=0; i<num_waveforms; i++) {
      const Fx3GpifWaveformCompat_t *w =
	&waveforms[waveform_select? waveform_select[i]:i];
      Fx3WriteReg32(FX3_GPIF_LEFT_WAVEFORM + i*16 + 0, w->left[0]);
      Fx3WriteReg32(FX3_GPIF_LEFT_WAVEFORM + i*16 + 4, w->left[1]);
      Fx3WriteReg32(FX3_GPIF_LEFT_WAVEFORM + i*16 + 8, w->left[2]);
      Fx3WriteReg32(FX3_GPIF_RIGHT_WAVEFORM + i*16 + 0, w->right[0]);
      Fx3WriteReg32(FX3_GPIF_RIGHT_WAVEFORM + i*16 + 4, w->right[1]);
      Fx3WriteReg32(FX3_GPIF_RIGHT_WAVEFORM + i*16 + 8, w->right[2]);
    }
  }
  Fx3GpifConfigureCommon(functions, num_functions, registers, num_registers);
}

void Fx3GpifPibStart(uint16_t clock_divisor_x2)
{
#ifdef DEBUG_UART
  {
    char _gpif_dbg[128];
    snprintf(_gpif_dbg, sizeof(_gpif_dbg), "Fx3GpifPibStart: clock_divisor_x2=%u\n", (unsigned)clock_divisor_x2);
    Fx3UartTxString(_gpif_dbg);
  }
#endif
  Fx3WriteReg32(FX3_GCTL_PIB_CORE_CLK,
		(((clock_divisor_x2 >> 1)-1) << FX3_GCTL_PIB_CORE_CLK_DIV_SHIFT) |
		(3UL << FX3_GCTL_PIB_CORE_CLK_SRC_SHIFT));
  if (clock_divisor_x2 & 1)
    Fx3SetReg32(FX3_GCTL_PIB_CORE_CLK, FX3_GCTL_PIB_CORE_CLK_HALFDIV);
  Fx3SetReg32(FX3_GCTL_PIB_CORE_CLK, FX3_GCTL_PIB_CORE_CLK_CLK_EN);

  Fx3WriteReg32(FX3_PIB_POWER, 0);
  Fx3UtilDelayUs(10);
  Fx3SetReg32(FX3_PIB_POWER, FX3_PIB_POWER_RESETN);
  while(!(Fx3ReadReg32(FX3_PIB_POWER) & FX3_PIB_POWER_ACTIVE))
    ;

  /* DLL frequency limits per Infineon docs:
   * - HIGH_FREQ=1: 70-230 MHz
   * - HIGH_FREQ=0: 23-80 MHz
   *
   * The DLL helps with clock/data alignment but appears to cause sporadic
   * data corruption at certain frequencies. Only enable DLL for high-speed
   * operation (>70 MHz) where it's most beneficial and well-characterized.
   *
   * With sys_clk ~480MHz: div_x2=14 gives ~69MHz (boundary for HIGH_FREQ)
   */
  if (clock_divisor_x2 <= 14) {
    /* High frequency (>~70 MHz) - enable DLL with HIGH_FREQ=1 */
    Fx3ClearReg32(FX3_PIB_DLL_CTRL, FX3_PIB_DLL_CTRL_ENABLE);
    Fx3UtilDelayUs(1);
    Fx3WriteReg32(FX3_PIB_DLL_CTRL,
		  FX3_PIB_DLL_CTRL_HIGH_FREQ | FX3_PIB_DLL_CTRL_ENABLE);
    Fx3UtilDelayUs(1);
    Fx3ClearReg32(FX3_PIB_DLL_CTRL, FX3_PIB_DLL_CTRL_DLL_RESET_N);
    Fx3UtilDelayUs(1);
    Fx3SetReg32(FX3_PIB_DLL_CTRL, FX3_PIB_DLL_CTRL_DLL_RESET_N);
    Fx3UtilDelayUs(1);
    while(!(Fx3ReadReg32(FX3_PIB_DLL_CTRL) & FX3_PIB_DLL_CTRL_DLL_STAT))
      ;
#ifdef DEBUG_UART
    Fx3UartTxString("Fx3GpifPibStart: DLL locked (HIGH_FREQ)\n");
#endif
  } else {
    /* Lower frequencies - disable DLL to avoid timing issues */
    Fx3WriteReg32(FX3_PIB_DLL_CTRL, 0);
#ifdef DEBUG_UART
    Fx3UartTxString("Fx3GpifPibStart: DLL disabled\n");
#endif
  }

  Fx3WriteReg32(FX3_VIC_VEC_ADDRESS + (FX3_IRQ_GPIF_CORE<<2), Fx3GpifPibIsr);
  Fx3WriteReg32(FX3_PIB_INTR, Fx3ReadReg32(FX3_PIB_INTR));
  /* Enable GPIF error, GPIF interrupt, and DLL lock loss interrupts */
  Fx3WriteReg32(FX3_PIB_INTR_MASK, FX3_PIB_INTR_MASK_GPIF_ERR |
		FX3_PIB_INTR_MASK_GPIF_INTERRUPT |
		FX3_PIB_INTR_MASK_DLL_LOST_LOCK);
  Fx3WriteReg32(FX3_VIC_INT_ENABLE, (1UL << FX3_IRQ_GPIF_CORE));
}

void Fx3GpifPibStop(void)
{
  Fx3WriteReg32(FX3_PIB_INTR_MASK, 0UL);
  Fx3WriteReg32(FX3_VIC_INT_CLEAR, (1UL << FX3_IRQ_GPIF_CORE));
  Fx3WriteReg32(FX3_PIB_INTR_MASK, 0UL);
  Fx3WriteReg32(FX3_PIB_INTR, ~0UL);
  Fx3WriteReg32(FX3_PIB_POWER, 0UL);
  Fx3UtilDelayUs(10);
  Fx3ClearReg32(FX3_GCTL_PIB_CORE_CLK, FX3_GCTL_PIB_CORE_CLK_CLK_EN);
}

void Fx3GpifSetClock(uint16_t clock_divisor_x2)
{
  /* Update PIB clock divider without full restart.
   * This allows changing frequency while GPIF is running. */

  /* Calculate new clock register value (preserve CLK_EN and SRC bits) */
  uint32_t clk_reg = (((clock_divisor_x2 >> 1)-1) << FX3_GCTL_PIB_CORE_CLK_DIV_SHIFT) |
                     (3UL << FX3_GCTL_PIB_CORE_CLK_SRC_SHIFT) |
                     FX3_GCTL_PIB_CORE_CLK_CLK_EN;
  if (clock_divisor_x2 & 1)
    clk_reg |= FX3_GCTL_PIB_CORE_CLK_HALFDIV;

  Fx3WriteReg32(FX3_GCTL_PIB_CORE_CLK, clk_reg);

  /* Re-configure DLL for new frequency if needed */
  if (clock_divisor_x2 <= 14) {
    /* High frequency (>~70 MHz) - enable DLL with HIGH_FREQ=1 */
    Fx3ClearReg32(FX3_PIB_DLL_CTRL, FX3_PIB_DLL_CTRL_ENABLE);
    Fx3UtilDelayUs(1);
    Fx3WriteReg32(FX3_PIB_DLL_CTRL,
                  FX3_PIB_DLL_CTRL_HIGH_FREQ | FX3_PIB_DLL_CTRL_ENABLE);
    Fx3UtilDelayUs(1);
    Fx3ClearReg32(FX3_PIB_DLL_CTRL, FX3_PIB_DLL_CTRL_DLL_RESET_N);
    Fx3UtilDelayUs(1);
    Fx3SetReg32(FX3_PIB_DLL_CTRL, FX3_PIB_DLL_CTRL_DLL_RESET_N);
    Fx3UtilDelayUs(1);
    while(!(Fx3ReadReg32(FX3_PIB_DLL_CTRL) & FX3_PIB_DLL_CTRL_DLL_STAT))
      ;
  } else {
    /* Lower frequencies - disable DLL */
    Fx3WriteReg32(FX3_PIB_DLL_CTRL, 0);
  }
}

Fx3GpifStat_t Fx3GpifGetStat(uint8_t *current_state)
{
  uint32_t stat = Fx3ReadReg32(FX3_GPIF_WAVEFORM_CTRL_STAT);
  if (current_state)
    *current_state =
      (stat & FX3_GPIF_WAVEFORM_CTRL_STAT_CURRENT_STATE_MASK)
      >> FX3_GPIF_WAVEFORM_CTRL_STAT_CURRENT_STATE_SHIFT;
  return (stat & FX3_GPIF_WAVEFORM_CTRL_STAT_GPIF_STAT_MASK)
    >> FX3_GPIF_WAVEFORM_CTRL_STAT_GPIF_STAT_SHIFT;
}
