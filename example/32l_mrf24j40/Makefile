TARGET=32l_mrf24j40
# all below here should not need to be touched...(much)


EXECUTABLE=$(TARGET).elf
BIN_IMAGE=$(TARGET).bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

DEBUG?=1
STM32_BASE=../..
STLINK_HOME=/home/karlp/src/stlink


CFLAGS=-Os -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m3
CFLAGS+=-ffreestanding -nostdlib -nostdinc
CFLAGS+=-Wall -Wextra -std=c99
ifeq ($(DEBUG),1)
        CFLAGS+=-g
endif

BOOT_FLASH?=1
ifeq ($(BOOT_FLASH),1)
	CFLAGS+=-Wl,-T,stm32l15xxb_flash.ld
else
	CFLAGS+=-Wl,-T,stm32l15xxb_sram.ld
	CFLAGS+=-DCONFIG_BOOT_SRAM=1
	# This keeps the standard system_xxx files from doing any damage...
	CFLAGS+=-DVECT_TAB_SRAM
endif

PLATFORM=stm32l1xx
LIBS_STM_PATH=$(STM32_BASE)/libs_stm

# stm32l_discovery lib
CFLAGS+=-I$(LIBS_STM_PATH)/inc/base
CFLAGS+=-I$(LIBS_STM_PATH)/inc/core_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/device_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/$(PLATFORM)

LDFLAGS+=-L$(LIBS_STM_PATH)/build -lstm32_stdperiph_l1xx
LDFLAGS+=-L$(STM32_BASE)/ld_scripts
LDFLAGS+=-Wl,-Map=$(TARGET).map

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

SRCS=main.c \
systick_ms.c \
lib_mrf24j.c \
mrf24j_stm32.c \
$(STM32_BASE)/startup/cm3_genstartup.c \
$(STM32_BASE)/startup/stm32l1xx_vectors.c \


$(EXECUTABLE): $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

write: all
	$(STLINK_HOME)/flash/flash write $(BIN_IMAGE) 0x08000000

.PHONY: all clean write
