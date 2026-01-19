/*
 * Copyright (C) 2026 Alessandro Carcione
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

#include <bsp/i2c.h>
#include <bsp/gctl.h>
#include <bsp/regaccess.h>
#include <bsp/util.h>
#include <rdb/i2c.h>
#include <rdb/gctl.h>
#include <rdb/lpp.h>

/*
 * 1. Use INTR register (not STATUS) for completion detection
 * 2. Read RX data byte-by-byte as it arrives (poll RX_DATA)
 * 3. No block reset between transactions - only on errors
 * 4. Proper error recovery sequence
 */

#define I2C_TIMEOUT_US 100000  /* 100ms timeout for I2C operations */

/* Clock source values for I2C_CORE_CLK register (same encoding for all LPP peripherals)
 *   0 = SYS_CLK / 16
 *   1 = SYS_CLK / 4
 *   2 = SYS_CLK / 2
 *   3 = SYS_CLK (direct)
 */
#define CLK_SRC_SYS_CLK_BY_16  0  /* SYS_CLK / 16 */
#define CLK_SRC_SYS_CLK_BY_4   1  /* SYS_CLK / 4 */
#define CLK_SRC_SYS_CLK_BY_2   2  /* SYS_CLK / 2 */
#define CLK_SRC_SYS_CLK        3  /* SYS_CLK direct */

/* Error flags to check in INTR register */
#define I2C_ERROR_FLAGS (FX3_I2C_INTR_ERROR | FX3_I2C_INTR_TIMEOUT | FX3_I2C_INTR_LOST_ARBITRATION)

static Fx3I2cSpeed_t i2c_speed = FX3_I2C_100KHZ;

/*
 * 1. Wait for block to be idle
 * 2. Disable block
 * 3. Clear FIFOs
 * 4. Re-enable block
 * 5. Clear interrupts
 */
static void Fx3I2cErrorRecovery(void)
{
  /* Wait for block to be idle */
  uint32_t timeout = 10000;
  while ((Fx3ReadReg32(FX3_I2C_STATUS) & FX3_I2C_STATUS_BUSY) && timeout--)
    Fx3UtilDelayUs(1);

  /* Save interrupt mask and disable interrupts */
  uint32_t mask = Fx3ReadReg32(FX3_I2C_INTR_MASK);
  Fx3WriteReg32(FX3_I2C_INTR_MASK, 0);
  Fx3WriteReg32(FX3_I2C_INTR, 0xFFFFFFFF);

  /* Disable block */
  uint32_t cfg = Fx3ReadReg32(FX3_I2C_CONFIG);
  Fx3WriteReg32(FX3_I2C_CONFIG, cfg & ~FX3_I2C_CONFIG_ENABLE);
  Fx3UtilDelayUs(5);

  /* Clear byte count */
  Fx3WriteReg32(FX3_I2C_BYTE_COUNT, 0);

  /* Clear FIFOs */
  Fx3WriteReg32(FX3_I2C_CONFIG, (cfg & ~FX3_I2C_CONFIG_ENABLE) |
                FX3_I2C_CONFIG_TX_CLEAR | FX3_I2C_CONFIG_RX_CLEAR);
  Fx3UtilDelayUs(1);

  /* Wait for FIFOs to clear */
  timeout = 10000;
  while (!(Fx3ReadReg32(FX3_I2C_STATUS) & FX3_I2C_STATUS_TX_DONE) && timeout--)
    Fx3UtilDelayUs(1);
  timeout = 10000;
  while ((Fx3ReadReg32(FX3_I2C_STATUS) & FX3_I2C_STATUS_RX_DATA) && timeout--)
    Fx3UtilDelayUs(1);

  /* Clear FIFO bits and re-enable block */
  Fx3WriteReg32(FX3_I2C_CONFIG, cfg | FX3_I2C_CONFIG_ENABLE);
  Fx3UtilDelayUs(5);

  /* Clear all interrupts and restore mask */
  Fx3WriteReg32(FX3_I2C_INTR, 0xFFFFFFFF);
  Fx3WriteReg32(FX3_I2C_INTR_MASK, mask);
}

/*
 * Light-weight clear - just clear flags and command, no FIFO clear
 * Used before starting a new transaction
 */
static void Fx3I2cClearState(void)
{
  /* Clear command register */
  Fx3WriteReg32(FX3_I2C_COMMAND, 0);

  /* Clear all interrupt flags */
  Fx3WriteReg32(FX3_I2C_INTR, 0xFFFFFFFF);

  /* Clear sticky status bits */
  Fx3WriteReg32(FX3_I2C_STATUS,
    FX3_I2C_STATUS_TIMEOUT | FX3_I2C_STATUS_LOST_ARBITRATION | FX3_I2C_STATUS_ERROR);
}

/*
 * Wait for bus to be free
 */
static Fx3I2cStatus_t Fx3I2cWaitBusFree(void)
{
  uint32_t timeout = 10000;
  while ((Fx3ReadReg32(FX3_I2C_STATUS) & (FX3_I2C_STATUS_BUS_BUSY | FX3_I2C_STATUS_BUSY)) && timeout--)
    Fx3UtilDelayUs(1);

  if (timeout == 0) {
    /* Bus stuck - try error recovery */
    Fx3I2cErrorRecovery();
    return I2C_RESULT_TIMEDOUT;
  }
  return I2C_RESULT_OK;
}

/*
 * Decode error from INTR/STATUS registers
 */
static Fx3I2cStatus_t Fx3I2cDecodeError(void)
{
  uint32_t status = Fx3ReadReg32(FX3_I2C_STATUS);
  uint32_t intr = Fx3ReadReg32(FX3_I2C_INTR);

  if (intr & FX3_I2C_INTR_TIMEOUT)
    return I2C_RESULT_TIMEDOUT;
  if (intr & FX3_I2C_INTR_LOST_ARBITRATION)
    return I2C_RESULT_ARB_LOST;
  if (intr & FX3_I2C_INTR_ERROR) {
    uint32_t err_code = (status & FX3_I2C_STATUS_ERROR_CODE_MASK) >> FX3_I2C_STATUS_ERROR_CODE_SHIFT;
    /* Error codes 0-7 = NACK on preamble byte N, 8 = NACK on data */
    if (err_code <= 8)
      return I2C_RESULT_NACK;
    return I2C_RESULT_ERROR;
  }
  return I2C_RESULT_ERROR;
}

void Fx3I2cInit(Fx3I2cSpeed_t speed)
{
  i2c_speed = speed;

  /* Step 1: Configure I2C core clock BEFORE powering up the I2C block.
   * The I2C clock must be enabled from GCTL before
   * resetting the I2C block from the I2C_POWER register."
   *
   * The I2C core clock should be 10x the bus clock rate:
   * - 400kHz bus -> 4MHz core clock
   * - 100kHz bus -> 1MHz core clock
   *
   * Using SYS_CLK/16 as source
   * SYS_CLK = 384MHz, SYS_CLK/16 = 24MHz
   * For 400kHz: need 4MHz core, div = 24/4 - 1 = 5
   * For 100kHz: need 1MHz core, div = 24/1 - 1 = 23
   */
  uint32_t target_core_clk = (speed == FX3_I2C_100KHZ) ? 1000000 : 4000000;
  uint32_t src_clk = SYS_CLK / 16;
  uint32_t div = (src_clk / target_core_clk) - 1;
  if (div < 1) div = 1;
  if (div > 0xFFF) div = 0xFFF;

  Fx3WriteReg32(FX3_GCTL_I2C_CORE_CLK,
    ((div << FX3_GCTL_I2C_CORE_CLK_DIV_SHIFT) & FX3_GCTL_I2C_CORE_CLK_DIV_MASK) |
    (CLK_SRC_SYS_CLK_BY_16 << FX3_GCTL_I2C_CORE_CLK_SRC_SHIFT) |
    FX3_GCTL_I2C_CORE_CLK_CLK_EN);

  /* Step 2: Initialize the LPP (Low Performance Peripheral) block if needed.
   * The I2C block is part of the LPP subsystem. The LPP block must be
   * powered on before individual peripherals can be used. */
  uint32_t lpp_power = Fx3ReadReg32(FX3_LPP_POWER);
  if (!(lpp_power & FX3_LPP_POWER_ACTIVE)) {
    /* Reset and enable the LPP block */
    Fx3WriteReg32(FX3_LPP_POWER, 0);  /* Clear RESETN */
    Fx3UtilDelayUs(10);
    Fx3WriteReg32(FX3_LPP_POWER, FX3_LPP_POWER_RESETN);
    while (!(Fx3ReadReg32(FX3_LPP_POWER) & FX3_LPP_POWER_ACTIVE))
      ;
  }

  /* Step 3: Reset and enable the I2C block */
  Fx3WriteReg32(FX3_I2C_POWER, 0);  /* Clear RESETN */
  Fx3UtilDelayUs(10);
  Fx3WriteReg32(FX3_I2C_POWER, FX3_I2C_POWER_RESETN);
  while (!(Fx3ReadReg32(FX3_I2C_POWER) & FX3_I2C_POWER_ACTIVE))
    ;

  /* Step 4: Configure I2C: enable, PIO mode, speed */
  uint32_t config = FX3_I2C_CONFIG_ENABLE;
  if (speed == FX3_I2C_100KHZ)
    config |= FX3_I2C_CONFIG_I2C_100KHz;

  Fx3WriteReg32(FX3_I2C_CONFIG, config);

  /* Set timeout (in I2C core clock cycles) */
  Fx3WriteReg32(FX3_I2C_TIMEOUT, 0xFFFFFFFF);

  /* Disable interrupts (we poll) */
  Fx3WriteReg32(FX3_I2C_INTR_MASK, 0);

  /* Clear any pending interrupts */
  Fx3WriteReg32(FX3_I2C_INTR, 0xFFFFFFFF);
}

Fx3I2cStatus_t Fx3I2cWrite(uint8_t slave_addr, const uint8_t *data, size_t len)
{
  if (len == 0 || len > 258)
    return I2C_RESULT_ERROR;

  /* Clear state (no full reset!) */
  Fx3I2cClearState();

  /* Wait for bus to be free */
  Fx3I2cStatus_t result = Fx3I2cWaitBusFree();
  if (result != I2C_RESULT_OK)
    return result;

  /* Disable preamble repeat */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_RPT, 0);

  /* Preamble: slave address with write bit (bit 0 = 0) */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_DATA, (slave_addr << 1) | 0);
  Fx3WriteReg32(FX3_I2C_PREAMBLE_DATA + 4, 0);

  /* Clear command register */
  Fx3WriteReg32(FX3_I2C_COMMAND, 0);

  /* Clear all interrupts */
  Fx3WriteReg32(FX3_I2C_INTR, 0xFFFFFFFF);

  /* Clear sticky status bits */
  Fx3WriteReg32(FX3_I2C_STATUS,
    FX3_I2C_STATUS_TIMEOUT | FX3_I2C_STATUS_LOST_ARBITRATION | FX3_I2C_STATUS_ERROR);

  /* Set byte count for data phase */
  Fx3WriteReg32(FX3_I2C_BYTE_COUNT, len);

  /* Preamble control: no additional START/STOP within preamble
   * START_FIRST in command register handles the initial START */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_CTRL, 0);

  /* Load first data byte into FIFO BEFORE sending command */
  Fx3WriteReg32(FX3_I2C_EGRESS_DATA, data[0]);

  /* Set command: 1 byte preamble, START_FIRST, STOP_LAST */
  uint32_t cmd = FX3_I2C_COMMAND_START_FIRST |
                 FX3_I2C_COMMAND_STOP_LAST |
                 (1UL << FX3_I2C_COMMAND_PREAMBLE_LEN_SHIFT);
  Fx3WriteReg32(FX3_I2C_COMMAND, cmd);

  /* Start the transaction by setting PREAMBLE_VALID (separate write per SDK) */
  Fx3WriteReg32(FX3_I2C_COMMAND, cmd | FX3_I2C_COMMAND_PREAMBLE_VALID);

  /* Feed remaining data bytes while checking for errors */
  for (size_t idx = 1; idx < len; idx++) {
    uint32_t timeout = I2C_TIMEOUT_US;

    /* Wait for TX_SPACE, checking errors */
    while (!(Fx3ReadReg32(FX3_I2C_STATUS) & FX3_I2C_STATUS_TX_SPACE)) {
      if (Fx3ReadReg32(FX3_I2C_INTR) & I2C_ERROR_FLAGS) {
        result = Fx3I2cDecodeError();
        Fx3I2cErrorRecovery();
        return result;
      }
      if (timeout-- == 0) {
        Fx3I2cErrorRecovery();
        return I2C_RESULT_TIMEDOUT;
      }
      Fx3UtilDelayUs(1);
    }

    /* Add next data byte */
    Fx3WriteReg32(FX3_I2C_EGRESS_DATA, data[idx]);
  }

  /* Wait for TX_DONE in INTR register */
  uint32_t timeout = I2C_TIMEOUT_US;
  while (!(Fx3ReadReg32(FX3_I2C_INTR) & FX3_I2C_INTR_TX_DONE)) {
    if (Fx3ReadReg32(FX3_I2C_INTR) & I2C_ERROR_FLAGS) {
      result = Fx3I2cDecodeError();
      Fx3I2cErrorRecovery();
      return result;
    }
    if (timeout-- == 0) {
      Fx3I2cErrorRecovery();
      return I2C_RESULT_TIMEDOUT;
    }
    Fx3UtilDelayUs(1);
  }

  return I2C_RESULT_OK;
}

Fx3I2cStatus_t Fx3I2cRead(uint8_t slave_addr, uint8_t *data, size_t len)
{
  if (len == 0 || len > 256)
    return I2C_RESULT_ERROR;

  /* Clear state (no full reset!) */
  Fx3I2cClearState();

  /* Wait for bus to be free */
  Fx3I2cStatus_t result = Fx3I2cWaitBusFree();
  if (result != I2C_RESULT_OK)
    return result;

  /* Disable preamble repeat */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_RPT, 0);

  /* Preamble: slave address with read bit (bit 0 = 1) */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_DATA, (slave_addr << 1) | 1);
  Fx3WriteReg32(FX3_I2C_PREAMBLE_DATA + 4, 0);

  /* Clear command register */
  Fx3WriteReg32(FX3_I2C_COMMAND, 0);

  /* Clear all interrupts */
  Fx3WriteReg32(FX3_I2C_INTR, 0xFFFFFFFF);

  /* Clear sticky status bits */
  Fx3WriteReg32(FX3_I2C_STATUS,
    FX3_I2C_STATUS_TIMEOUT | FX3_I2C_STATUS_LOST_ARBITRATION | FX3_I2C_STATUS_ERROR);

  /* Set byte count */
  Fx3WriteReg32(FX3_I2C_BYTE_COUNT, len);

  /* Preamble control: no additional START/STOP within preamble
   * START_FIRST in command register handles the initial START */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_CTRL, 0);

  /* Set command: READ, 1 byte preamble, START_FIRST, STOP_LAST, NAK_LAST */
  uint32_t cmd = FX3_I2C_COMMAND_READ |
                 FX3_I2C_COMMAND_START_FIRST |
                 FX3_I2C_COMMAND_STOP_LAST |
                 FX3_I2C_COMMAND_NAK_LAST |
                 (1UL << FX3_I2C_COMMAND_PREAMBLE_LEN_SHIFT);
  Fx3WriteReg32(FX3_I2C_COMMAND, cmd);

  /* Start the transaction by setting PREAMBLE_VALID (separate write per SDK) */
  Fx3WriteReg32(FX3_I2C_COMMAND, cmd | FX3_I2C_COMMAND_PREAMBLE_VALID);

  /* Read bytes as they arrive */
  for (size_t idx = 0; idx < len; idx++) {
    uint32_t timeout = I2C_TIMEOUT_US;

    /* Wait for RX_DATA, checking errors */
    while (!(Fx3ReadReg32(FX3_I2C_STATUS) & FX3_I2C_STATUS_RX_DATA)) {
      if (Fx3ReadReg32(FX3_I2C_INTR) & I2C_ERROR_FLAGS) {
        result = Fx3I2cDecodeError();
        Fx3I2cErrorRecovery();
        return result;
      }
      if (timeout-- == 0) {
        Fx3I2cErrorRecovery();
        return I2C_RESULT_TIMEDOUT;
      }
      Fx3UtilDelayUs(1);
    }

    /* Read byte immediately */
    data[idx] = Fx3ReadReg32(FX3_I2C_INGRESS_DATA) & 0xFF;
  }

  return I2C_RESULT_OK;
}

Fx3I2cStatus_t Fx3I2cWriteRead(uint8_t slave_addr,
                                const uint8_t *write_data, size_t write_len,
                                uint8_t *read_data, size_t read_len)
{
  if (write_len == 0 || write_len > 7 || read_len == 0 || read_len > 64)
    return I2C_RESULT_ERROR;

  /* Clear state */
  Fx3I2cClearState();

  /* Wait for bus to be free */
  Fx3I2cStatus_t result = Fx3I2cWaitBusFree();
  if (result != I2C_RESULT_OK)
    return result;

  /* Disable preamble repeat */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_RPT, 0);

  /* Build preamble: addr+W, write_data bytes, addr+R
   * Preamble can hold up to 8 bytes. */
  uint64_t preamble = 0;
  size_t preamble_len = 0;

  /* Byte 0: slave address with write bit */
  preamble |= ((uint64_t)((slave_addr << 1) | 0)) << (preamble_len * 8);
  preamble_len++;

  /* Bytes 1..N: write data (register address, etc.) */
  for (size_t i = 0; i < write_len; i++) {
    preamble |= ((uint64_t)write_data[i]) << (preamble_len * 8);
    preamble_len++;
  }

  /* Final byte: slave address with read bit */
  preamble |= ((uint64_t)((slave_addr << 1) | 1)) << (preamble_len * 8);
  preamble_len++;

  /* Write preamble data (64-bit register) */
  Fx3WriteReg32(FX3_I2C_PREAMBLE_DATA, preamble & 0xFFFFFFFF);
  Fx3WriteReg32(FX3_I2C_PREAMBLE_DATA + 4, (preamble >> 32) & 0xFFFFFFFF);

  /* Clear command register */
  Fx3WriteReg32(FX3_I2C_COMMAND, 0);

  /* Clear all interrupts */
  Fx3WriteReg32(FX3_I2C_INTR, 0xFFFFFFFF);

  /* Clear sticky status bits */
  Fx3WriteReg32(FX3_I2C_STATUS,
    FX3_I2C_STATUS_TIMEOUT | FX3_I2C_STATUS_LOST_ARBITRATION | FX3_I2C_STATUS_ERROR);

  /* Set byte count for read phase */
  Fx3WriteReg32(FX3_I2C_BYTE_COUNT, read_len);

  /* Preamble control:
   * START at byte 0 (first address)
   * START at last byte (repeated start before read address)
   * Bit N in START field means START condition before byte N */
  uint8_t start_bits = 0x01 | (1 << (preamble_len - 1));

  Fx3WriteReg32(FX3_I2C_PREAMBLE_CTRL,
    (0x00UL << FX3_I2C_PREAMBLE_CTRL_STOP_SHIFT) |
    ((uint32_t)start_bits << FX3_I2C_PREAMBLE_CTRL_START_SHIFT));

  /* Set command: READ, preamble_len bytes, START_FIRST, STOP_LAST, NAK_LAST */
  uint32_t cmd = FX3_I2C_COMMAND_READ |
                 FX3_I2C_COMMAND_START_FIRST |
                 FX3_I2C_COMMAND_STOP_LAST |
                 FX3_I2C_COMMAND_NAK_LAST |
                 (preamble_len << FX3_I2C_COMMAND_PREAMBLE_LEN_SHIFT);
  Fx3WriteReg32(FX3_I2C_COMMAND, cmd);

  /* Start the transaction by setting PREAMBLE_VALID (separate write per SDK) */
  Fx3WriteReg32(FX3_I2C_COMMAND, cmd | FX3_I2C_COMMAND_PREAMBLE_VALID);

  /* Read bytes as they arrive */
  for (size_t idx = 0; idx < read_len; idx++) {
    uint32_t timeout = I2C_TIMEOUT_US;

    /* Wait for RX_DATA, checking errors */
    while (!(Fx3ReadReg32(FX3_I2C_STATUS) & FX3_I2C_STATUS_RX_DATA)) {
      if (Fx3ReadReg32(FX3_I2C_INTR) & I2C_ERROR_FLAGS) {
        result = Fx3I2cDecodeError();
        Fx3I2cErrorRecovery();
        return result;
      }
      if (timeout-- == 0) {
        Fx3I2cErrorRecovery();
        return I2C_RESULT_TIMEDOUT;
      }
      Fx3UtilDelayUs(1);
    }

    /* Read byte immediately */
    read_data[idx] = Fx3ReadReg32(FX3_I2C_INGRESS_DATA) & 0xFF;
  }

  return I2C_RESULT_OK;
}
