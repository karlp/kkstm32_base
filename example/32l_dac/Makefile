EXECUTABLE=dac.elf
BIN_IMAGE=dac.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-O3 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m3 -DCONFIG_STM32L_DISCOVERY=1
CFLAGS+=-ffreestanding -nostdlib -nostdinc

# to run from FLASH
CFLAGS+=-Wl,-T,stm32l15xxb_flash.ld

PLATFORM=stm32l1xx
STM32_BASE=../..

LIBS_STM_PATH=$(STM32_BASE)/libs_stm

# stm32l_discovery lib
CFLAGS+=-I$(LIBS_STM_PATH)/inc/base
CFLAGS+=-I$(LIBS_STM_PATH)/inc/core_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/device_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/$(PLATFORM)

LDFLAGS+=-L$(STM32_BASE)/ld_scripts
LDFLAGS+=-L$(LIBS_STM_PATH)/build -lstm32_stdperiph_l1xx

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c system_stm32l1xx.c $(STM32_BASE)/startup/cm3_genstartup.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

write: all
	../../flash/flash write $(BIN_IMAGE) 0x08000000

.PHONY: all clean write
