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

#ifndef BSP_I2C_H_
#define BSP_I2C_H_

#include <stdint.h>
#include <stddef.h>

typedef enum {
  FX3_I2C_100KHZ = 0,
  FX3_I2C_400KHZ = 1,
} Fx3I2cSpeed_t;

typedef enum {
  I2C_RESULT_OK = 0,
  I2C_RESULT_NACK = 1,
  I2C_RESULT_TIMEDOUT = 2,
  I2C_RESULT_ARB_LOST = 3,
  I2C_RESULT_ERROR = 4,
} Fx3I2cStatus_t;

/* Initialize I2C peripheral with specified speed */
extern void Fx3I2cInit(Fx3I2cSpeed_t speed);

/* Write bytes to I2C device at 7-bit address */
extern Fx3I2cStatus_t Fx3I2cWrite(uint8_t slave_addr, const uint8_t *data, size_t len);

/* Read bytes from I2C device at 7-bit address */
extern Fx3I2cStatus_t Fx3I2cRead(uint8_t slave_addr, uint8_t *data, size_t len);

/* Combined write-then-read with repeated start (for register reads) */
extern Fx3I2cStatus_t Fx3I2cWriteRead(uint8_t slave_addr,
                                       const uint8_t *write_data, size_t write_len,
                                       uint8_t *read_data, size_t read_len);

#endif /* BSP_I2C_H_ */
