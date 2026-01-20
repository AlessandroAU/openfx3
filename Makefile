MAKEFLAGS += -j$(shell nproc 2>/dev/null || echo 4)

CC = arm-none-eabi-gcc
PYTHON := $(shell python3 --version >/dev/null 2>&1 && echo python3 || echo python)

BUILD = build
CGENFLAGS = -mcpu=arm926ej-s -mthumb-interwork -fno-pie
WARN = -Wall -Wextra -Werror -Wno-unused-function
OPTIMIZE = -g -Os
INCLUDE = -Isrc -Isrc/bsp -Isrc/rdb
GENDEP = -MMD -MP

CFLAGS = -std=c11 $(CGENFLAGS) $(WARN) $(OPTIMIZE) $(INCLUDE) $(GENDEP) $(DEFS) -fno-stack-protector -ffunction-sections -fdata-sections
LDFLAGS = -Wl,--gc-sections -static -nostartfiles -T src/bsp/fx3.ld -Wl,-z,max-page-size=4096,-Map,$(BUILD)/openfx3.map

VPATH = src src/bsp

OBJS = main.o descriptors.o acquisition.o benchmark.o usb.o gpif.o gctl.o gpio.o uart.o i2c.o util.o dma.o dma_pool.o irq.o cache.o vectors.o
BUILD_OBJS = $(addprefix $(BUILD)/,$(OBJS))

all : $(BUILD)/openfx3.fw examples

$(BUILD)/openfx3.fw : $(BUILD)/openfx3.elf
	$(PYTHON) elf2img.py $< $@

$(BUILD)/openfx3.elf : $(BUILD_OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(BUILD)/%.o : %.c | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/%.o : %.S | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD) :
	mkdir -p $@

clean :
	rm -rf $(BUILD)
	rm -f *.exe stream sweep benchmark i2cscan i2ctest i2cdump i2cprogram fpga_config

-include $(BUILD_OBJS:.o=.d)

# ==============================================================================
# Host library and examples (native compiler, not cross-compiled)
# ==============================================================================

HOST_CC = gcc
HOST_AR = ar
HOST_CFLAGS = -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -O3 -g -Ilib/include $(shell pkg-config --cflags libusb-1.0)
HOST_LDFLAGS = $(shell pkg-config --libs libusb-1.0)

# Embedded firmware header (RLE compressed)
lib/src/firmware.h : $(BUILD)/openfx3.fw fw2header.py
	$(PYTHON) fw2header.py $< $@

# ------------------------------------------------------------------------------
# Static library: libopenfx3.a
# Contains all reusable host-side code for FX3 device management
#
# Usage:
#   gcc -o myapp myapp.c -Ilib/include -Lbuild -lopenfx3 $(pkg-config --libs libusb-1.0)
# ------------------------------------------------------------------------------

LIBOPENFX3_OBJS = $(BUILD)/lib_device.o $(BUILD)/lib_acquisition.o
LIBOPENFX3 = $(BUILD)/libopenfx3.a

lib : $(LIBOPENFX3)

$(LIBOPENFX3) : $(LIBOPENFX3_OBJS)
	$(HOST_AR) rcs $@ $^

# Install headers and library to a prefix (default: /usr/local)
PREFIX ?= /usr/local
install-lib : $(LIBOPENFX3)
	install -d $(PREFIX)/lib
	install -d $(PREFIX)/include/openfx3
	install -m 644 $(LIBOPENFX3) $(PREFIX)/lib/
	install -m 644 lib/include/openfx3/*.h $(PREFIX)/include/openfx3/

# Library object files (prefixed with lib_ to avoid collision with firmware objects)
$(BUILD)/lib_device.o : lib/src/device.c lib/src/firmware.h | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -Ilib/src -c -o $@ $<

$(BUILD)/lib_acquisition.o : lib/src/acquisition.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

# ------------------------------------------------------------------------------
# Example applications (link against libopenfx3.a)
# Prefixed with ex_ to avoid collision with firmware objects
# ------------------------------------------------------------------------------

examples : $(BUILD)/stream $(BUILD)/sweep $(BUILD)/benchmark $(BUILD)/i2cscan $(BUILD)/i2ctest $(BUILD)/i2cdump $(BUILD)/i2cprogram $(BUILD)/fpga_config

$(BUILD)/stream : $(BUILD)/ex_stream.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

$(BUILD)/sweep : $(BUILD)/ex_sweep.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

$(BUILD)/benchmark : $(BUILD)/ex_benchmark.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

# Example object files
$(BUILD)/ex_stream.o : examples/stream.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

$(BUILD)/ex_sweep.o : examples/sweep.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

$(BUILD)/ex_benchmark.o : examples/benchmark.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

$(BUILD)/i2cscan : $(BUILD)/ex_i2cscan.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

$(BUILD)/ex_i2cscan.o : examples/i2cscan.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

$(BUILD)/i2ctest : $(BUILD)/ex_i2ctest.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

$(BUILD)/ex_i2ctest.o : examples/i2ctest.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

$(BUILD)/i2cdump : $(BUILD)/ex_i2cdump.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

$(BUILD)/ex_i2cdump.o : examples/i2cdump.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

$(BUILD)/i2cprogram : $(BUILD)/ex_i2cprogram.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

$(BUILD)/ex_i2cprogram.o : examples/i2cprogram.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<

$(BUILD)/fpga_config : $(BUILD)/ex_fpga_config.o $(LIBOPENFX3)
	$(HOST_CC) -o $@ $< -L$(BUILD) -lopenfx3 $(HOST_LDFLAGS)
	cp $@ .

$(BUILD)/ex_fpga_config.o : examples/fpga_config.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -c -o $@ $<
