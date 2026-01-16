
#include <bsp/gctl.h>
#include <bsp/gpio.h>
#include <bsp/gpif.h>
#include <bsp/uart.h>
#include <bsp/usb.h>
#include <bsp/irq.h>
#include <bsp/cache.h>
#include <bsp/util.h>
#include <bsp/regaccess.h>
#include <rdb/uib.h>

#include <string.h>
#include <stdio.h>

#include "descriptors.h"
#include "acquisition.h"
#include "benchmark.h"
#include "command.h"


static volatile uint8_t DmaBuf[64] __attribute__((aligned(32)));

static void VendorCommand(uint8_t request_type, uint8_t request, uint16_t value,
			  uint16_t index, uint16_t length, Fx3UsbSpeed_t s)
{
  switch(request) {
  case CMD_START:
    if (request_type !=
	(FX3_USB_REQTYPE_OUT | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    /* Command format:
     * wValue bit 0: clock source (0=external, 1=internal)
     * wIndex: must be 0
     * Data: struct cmd_start_acquisition (3 bytes) */
    if (index != 0 || length != sizeof(struct cmd_start_acquisition))
      goto stall;
    Fx3UartTxString("CMD_START\n");
    Fx3UsbUnstallEp0(s);
    Fx3UsbDmaDataOut(0, DmaBuf, length);
    Fx3CacheInvalidateDCacheEntry(DmaBuf);

    volatile struct cmd_start_acquisition *cmd = (volatile struct cmd_start_acquisition *)DmaBuf;
    uint8_t bus_mhz = cmd->bus_mhz;
    uint8_t bus_width = cmd->bus_width;
    uint8_t use_internal_clock = (value != 0) ? 1 : 0;

    /* Validate bus_width */
    if (bus_width != 8 && bus_width != 16 && bus_width != 24 && bus_width != 32)
      bus_width = 32;

    /* bus_mhz=0 means external clocking */
    if (bus_mhz == 0)
      use_internal_clock = 0;

    /* Configure clock for best match to requested bus speed */
    Fx3ClockConfig_t clk_cfg;
    uint16_t clock_divisor_x2 = 2;  /* Default minimum */

    if (use_internal_clock && bus_mhz > 0) {
      /* Set PLL to best value for requested bus speed */
      Fx3GctlSetBusClock(bus_mhz, &clk_cfg);
      /* gpif_div is the raw divider register value where:
       * bus_clk = sys_clk * 2 / (gpif_div + 1)
       * Fx3GpifPibStart expects clock_divisor_x2 = gpif_div + 1 */
      clock_divisor_x2 = clk_cfg.gpif_div + 1;
      if (clock_divisor_x2 < 2)
        clock_divisor_x2 = 2;

      char buf[80];
      snprintf(buf, sizeof(buf), "internal clock, %u-bit, actual=%lu.%02luMHz\n",
               (unsigned)bus_width,
               (unsigned long)(clk_cfg.bus_clk_hz / 1000000),
               (unsigned long)((clk_cfg.bus_clk_hz % 1000000) / 10000));
      Fx3UartTxString(buf);
      /* Store bus config for status queries */
      acq_bus_freq_hz = clk_cfg.bus_clk_hz;
      acq_sys_clk_hz = clk_cfg.sys_clk_hz;
      acq_gpif_div = clk_cfg.gpif_div;
      acq_pll_fbdiv = clk_cfg.pll_fbdiv;
      acq_bus_width = bus_width;
    } else {
      char buf[80];
      snprintf(buf, sizeof(buf), "external clock, %u-bit\n", (unsigned)bus_width);
      Fx3UartTxString(buf);

      /* Store bus config for status queries (0 = external clock) */
      acq_bus_freq_hz = 0;
      acq_sys_clk_hz = 0;
      acq_gpif_div = 0;
      acq_pll_fbdiv = 0;
      acq_bus_width = bus_width;
    }
    start_acquisition(bus_width, clock_divisor_x2, use_internal_clock);
    return;
  case CMD_GET_FW_VERSION:
    if (request_type !=
	(FX3_USB_REQTYPE_IN | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    if (value != 0 || index != 0 || length != sizeof(struct version_info))
      goto stall;
    Fx3UartTxString("CMD_GET_FW_VERSION\n");
    volatile struct version_info *vinfo = (volatile struct version_info *)DmaBuf;
    vinfo->major = 1;
    vinfo->minor = 3;
    Fx3CacheCleanDCacheEntry(DmaBuf);
    Fx3UsbUnstallEp0(s);
    Fx3UsbDmaDataIn(0, DmaBuf, sizeof(struct version_info));
    return;
  case CMD_GET_FW_MODE:
    if (request_type !=
	(FX3_USB_REQTYPE_IN | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    if (value != 0 || index != 0 || length != 1)
      goto stall;
    Fx3UartTxString("CMD_GET_FW_MODE\n");
    DmaBuf[0] = 1;
    Fx3CacheCleanDCacheEntry(DmaBuf);
    Fx3UsbUnstallEp0(s);
    Fx3UsbDmaDataIn(0, DmaBuf, 1);
    return;
  case CMD_STOP:
    if (request_type !=
	(FX3_USB_REQTYPE_OUT | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    Fx3UartTxString("CMD_STOP\n");
    stop_acquisition();
    Fx3UsbUnstallEp0(s);
    return;
  case CMD_GET_DROP_STATS:
    if (request_type !=
	(FX3_USB_REQTYPE_IN | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    if (value != 0 || index != 0 || length != sizeof(struct cmd_drop_stats))
      goto stall;
    Fx3UartTxString("CMD_GET_DROP_STATS\n");
    {
      volatile struct cmd_drop_stats *stats = (volatile struct cmd_drop_stats *)DmaBuf;
      stats->total_buffers = acq_total_buffers;
      stats->total_bytes = get_acq_total_bytes();
      stats->stalls = acq_stalls;
      stats->gpif_errors = gpif_error_count;
      Fx3CacheCleanDCacheEntry(DmaBuf);
      Fx3UsbUnstallEp0(s);
      Fx3UsbDmaDataIn(0, DmaBuf, sizeof(struct cmd_drop_stats));
    }
    return;
  case CMD_RESET_DROP_STATS:
    if (request_type !=
	(FX3_USB_REQTYPE_OUT | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    if (value != 0 || index != 0 || length != 0)
      goto stall;
    Fx3UartTxString("CMD_RESET_DROP_STATS\n");
    reset_drop_stats();
    Fx3UsbUnstallEp0(s);
    return;
  case CMD_GET_ACQ_STATUS:
    if (request_type !=
	(FX3_USB_REQTYPE_IN | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    if (value != 0 || index != 0 || length != sizeof(struct cmd_acq_status))
      goto stall;
    Fx3UartTxString("CMD_GET_ACQ_STATUS\n");
    {
      volatile struct cmd_acq_status *status = (volatile struct cmd_acq_status *)DmaBuf;
      /* Active if either acquisition or benchmark is running */
      status->active = is_acquisition_active() || is_benchmark_active();
      status->bus_width = acq_bus_width;
      status->pll_fbdiv = acq_pll_fbdiv;
      status->reserved = 0;
      status->bus_freq_hz = acq_bus_freq_hz;
      status->sys_clk_hz = acq_sys_clk_hz;
      status->gpif_div = acq_gpif_div;
      status->reserved2 = 0;
    }
    Fx3CacheCleanDCacheEntry(DmaBuf);
    Fx3UsbUnstallEp0(s);
    Fx3UsbDmaDataIn(0, DmaBuf, sizeof(struct cmd_acq_status));
    return;
  case CMD_REBOOT_BOOTLOADER:
    if (request_type !=
	(FX3_USB_REQTYPE_OUT | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    if (value != 0 || index != 0 || length != 0)
      goto stall;
    Fx3UartTxString("CMD_REBOOT_BOOTLOADER\n");
    Fx3UartTxFlush();
    /* Stop acquisition if running */
    stop_acquisition();
    /* Hard reset to bootloader */
    Fx3GctlHardReset();
    /* Never returns */
    return;
  case CMD_START_BENCHMARK:
    if (request_type !=
	(FX3_USB_REQTYPE_OUT | FX3_USB_REQTYPE_TYPE_VENDOR | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    /* No data phase needed - just start benchmark mode */
    if (value != 0 || index != 0 || length != 0)
      goto stall;
    Fx3UartTxString("CMD_START_BENCHMARK\n");
    Fx3UsbUnstallEp0(s);
    start_benchmark();
    return;
  }
 stall:
  Fx3UsbStallEp0(s);
}

static void SetupData(uint8_t request_type, uint8_t request, uint16_t value,
		      uint16_t index, uint16_t length, Fx3UsbSpeed_t s)
{
  char buf[64];
  snprintf(buf, sizeof(buf),
	   "req: %02x %02x value: %04x index: %04x length: %04x\n",
	   (unsigned)request_type, (unsigned)request,
	   (unsigned)value, (unsigned)index, (unsigned)length);
  Fx3UartTxString(buf);

  if ((request_type & FX3_USB_REQTYPE_TYPE_MASK) == FX3_USB_REQTYPE_TYPE_VENDOR) {
    VendorCommand(request_type, request, value, index, length, s);
    return;
  }

  if ((request_type & FX3_USB_REQTYPE_TYPE_MASK) != FX3_USB_REQTYPE_TYPE_STD)
    goto stall;

  switch(request) {
  case FX3_USB_STD_REQUEST_GET_DESCRIPTOR:
    if (request_type !=
	(FX3_USB_REQTYPE_IN | FX3_USB_REQTYPE_TYPE_STD | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    const uint8_t *descr = GetDescriptor(value>>8, value&0xff, s);
    if (!descr) goto stall;
    uint8_t descr_type = descr[1];
    uint16_t len = (descr_type == FX3_USB_DESCRIPTOR_CONFIGURATION ||
		    descr_type == FX3_USB_DESCRIPTOR_BOS?
		    *(const uint16_t *)(descr+2) : *descr);
    if (len < length)
      length = len;
    Fx3UsbUnstallEp0(s);
    Fx3UsbDmaDataIn(0, descr, length);
    return;

  case FX3_USB_STD_REQUEST_SET_CONFIGURATION:
    if (request_type !=
	(FX3_USB_REQTYPE_OUT | FX3_USB_REQTYPE_TYPE_STD | FX3_USB_REQTYPE_TGT_DEVICE))
      goto stall;
    if (value != 1)
      goto stall;

    /* Enable endpoint with appropriate packet size and burst length */
    if (s == FX3_USB_SUPER_SPEED) {
      Fx3UsbEnableInEndpoint(2, FX3_USB_EP_BULK, 1024, 16);  /* SuperSpeed: 16-packet burst */
    } else {
      Fx3UsbEnableInEndpoint(2, FX3_USB_EP_BULK, 512, 1);    /* HighSpeed: no burst */
    }
    Fx3UsbUnstallEp0(s);
    return;

  case FX3_USB_STD_REQUEST_GET_STATUS:
    /* Return device/interface/endpoint status */
    if ((request_type & FX3_USB_REQTYPE_DIR_MASK) != FX3_USB_REQTYPE_IN)
      goto stall;
    if (length < 2)
      goto stall;
    /* Return 0x0000 status (not self-powered, no remote wakeup, no U1/U2 enabled) */
    DmaBuf[0] = 0;
    DmaBuf[1] = 0;
    Fx3CacheCleanDCacheEntry(DmaBuf);
    Fx3UsbUnstallEp0(s);
    Fx3UsbDmaDataIn(0, DmaBuf, 2);
    return;

  case FX3_USB_STD_REQUEST_SET_FEATURE:
  case FX3_USB_STD_REQUEST_CLEAR_FEATURE:
    /* Handle feature requests for device, interface, or endpoint */
    if ((request_type & FX3_USB_REQTYPE_DIR_MASK) != FX3_USB_REQTYPE_OUT)
      goto stall;

    /* Check if this is an endpoint-targeted CLEAR_FEATURE */
    if ((request_type & FX3_USB_REQTYPE_TGT_MASK) == FX3_USB_REQTYPE_TGT_EP &&
        value == 0 && request == FX3_USB_STD_REQUEST_CLEAR_FEATURE) {
      uint8_t ep_num = index & 0x0F;
      if (ep_num == 2) {
        Fx3UartTxString("CLEAR_FEATURE EP2\n");
        /* Minimal approach: Don't touch DMA at all.
         *
         * Our GPIF is continuously sampling data into DMA buffers which
         * auto-forward to USB. The USB endpoint just needs its sequence
         * number reset so the host can start requesting data again.
         *
         * Touching DMA while GPIF is actively running causes state machine
         * corruption - the GPIF gets stuck waiting for DMA_RDY that never
         * comes back properly.
         */
        Fx3UsbSetEpNak(ep_num, 1);
        Fx3UtilDelayUs(125);

        /* Just flush the USB endpoint and reset sequence number */
        Fx3UsbFlushInEndpoint(ep_num);

        /* Reset endpoint sequence number */
        Fx3SetReg32(FX3_PROT_EPI_CS1+(ep_num<<2), FX3_PROT_EPI_CS1_EP_RESET);
        Fx3UtilDelayUs(1);
        Fx3ClearReg32(FX3_PROT_EPI_CS1+(ep_num<<2), FX3_PROT_EPI_CS1_EP_RESET);
        Fx3SetReg32(FX3_PROT_EPI_CS1+(ep_num<<2), FX3_PROT_EPI_CS1_VALID);

        /* Clear NAK to resume transfers */
        Fx3UsbSetEpNak(ep_num, 0);
      }
    }
    Fx3UsbUnstallEp0(s);
    return;

  case FX3_USB_STD_REQUEST_SET_SEL:
    /* USB 3.0 Set System Exit Latency - host sends 6 bytes of SEL data */
    if ((request_type & FX3_USB_REQTYPE_DIR_MASK) != FX3_USB_REQTYPE_OUT)
      goto stall;
    /* Accept the data (we don't use it, but must receive it) */
    if (length > 0) {
      Fx3UsbUnstallEp0(s);
      Fx3UsbDmaDataOut(0, DmaBuf, length > 6 ? 6 : length);
    } else {
      Fx3UsbUnstallEp0(s);
    }
    return;

  case FX3_USB_STD_REQUEST_SET_ISOCH_DELAY:
    /* USB 3.0 Set Isochronous Delay - no data phase, just ACK */
    if ((request_type & FX3_USB_REQTYPE_DIR_MASK) != FX3_USB_REQTYPE_OUT)
      goto stall;
    Fx3UsbUnstallEp0(s);
    return;
  }

 stall:
  Fx3UsbStallEp0(s);
}



int main(void)
{
  static const struct Fx3UsbCallbacks callbacks = {
    .sutok = SetupData
  };

  Fx3CacheEnableCaches();
  Fx3IrqInit();

  Fx3GctlInitClock();
  *(volatile uint32_t *)(void *)0x400020e8 = 0;
  Fx3GctlInitIoMatrix(FX3_GCTL_ALTFUNC_GPIF32BIT_UART_I2S);
  Fx3UartInit(115200, FX3_UART_NO_PARITY, FX3_UART_1_STOP_BIT);

  /* One-time clock source debug: show FSLC and derived PLL reference clock. */
  {
    uint32_t fslc = Fx3GctlGetPllFslc();
    uint32_t ref_hz = Fx3GctlGetPllRefClkHz();

    char clkbuf[96];
    snprintf(clkbuf, sizeof(clkbuf), "PLL ref: FSLC=0x%lx -> %lu Hz\n",
             (unsigned long)fslc, (unsigned long)ref_hz);
    Fx3UartTxString(clkbuf);
  }

  Fx3UartTxString("\nGood moaning!\n");
  Fx3UartTxFlush();

  // Fx3GpioInitClock();
  // Fx3GpioSetupSimple(45,
	// 	     FX3_GPIO_SIMPLE_ENABLE |
	// 	     FX3_GPIO_SIMPLE_INPUT_EN);
  // Fx3GpioSetupSimple(54,
	// 	     FX3_GPIO_SIMPLE_ENABLE |
	// 	     FX3_GPIO_SIMPLE_DRIVE_HI_EN |
	// 	     FX3_GPIO_SIMPLE_DRIVE_LO_EN);

  // // Divide the GPIO slow clock into a 1kHz clock output for reference
  // Fx3GpioSetupComplex(50,
	// 	      FX3_PIN_STATUS_ENABLE |
	// 	      (FX3_GPIO_TIMER_MODE_SLOW_CLK << FX3_PIN_STATUS_TIMER_MODE_SHIFT) |
	// 	      (FX3_GPIO_PIN_MODE_PWM << FX3_PIN_STATUS_MODE_SHIFT) |
	// 	      FX3_PIN_STATUS_DRIVE_HI_EN |
	// 	      FX3_PIN_STATUS_DRIVE_LO_EN,
	// 	      0, GPIO_SLOW_CLK/1000-1, GPIO_SLOW_CLK/2000);

  Fx3IrqEnableInterrupts();
  Fx3UsbInit(&callbacks);
  Fx3UsbConnect();

  /* Timing counters for different polling intervals */
  uint32_t acquisition_timer = 0;
  const uint32_t ACQUISITION_INTERVAL_US = 1000000; /* 1000ms */
  const uint32_t LOOP_DELAY_US = 10000;              /* Base loop delay */

  for(;;) {
    /* poll_acquisition every 500ms */
    if (acquisition_timer >= ACQUISITION_INTERVAL_US) {
      poll_acquisition();
      acquisition_timer = 0;
    }

    Fx3UtilDelayUs(LOOP_DELAY_US);
    acquisition_timer += LOOP_DELAY_US;
  }
}

/* Workaround for handling newlib being compiled with ssp */
uintptr_t __stack_chk_guard = 0x00000aff;
void __stack_chk_fail(void)
{
}

/* Newlib's assert() calls this function if the assertion fails */
void
__assert_func (const char *file,
        int line,
        const char *func,
        const char *failedexpr)
{
  if (file != NULL) {
    char linestrbuf[16], *linestr = &linestrbuf[sizeof(linestrbuf)];
    Fx3UartTxString(file);
    Fx3UartTxChar(':');
    /* Avoid using newlib functions like itoa so as not to trigger
       a recursive assert... */
    *--linestr = '\0';
    while (line >= 10 && linestr != &linestrbuf[1]) {
      *--linestr = '0' + (line % 10);
      line /= 10;
    }
    *--linestr = '0' + line;
    Fx3UartTxString(linestr);
    Fx3UartTxString(": ");
  }
  if (func != NULL) {
    Fx3UartTxString(func);
    Fx3UartTxString(": ");
  }
  Fx3UartTxString("Assertion ");
  if (failedexpr != NULL) {
    Fx3UartTxChar('`');
    Fx3UartTxString(failedexpr);
    Fx3UartTxString("' ");
  }
  Fx3UartTxString("failed.\n");
  for(;;)
    ;
}
