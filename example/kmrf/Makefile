TARGET=kmrf
# all below here should not need to be touched...(much)


EXECUTABLE=$(TARGET).elf
BIN_IMAGE=$(TARGET).bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

DEBUG?=1
CFLAGS=-Os -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m3
CFLAGS+=-ffreestanding -nostdlib -nostdinc
CFLAGS+=-Wall -Wextra
ifeq ($(DEBUG),1)
        CFLAGS+=-g
endif

# to run from FLASH
CFLAGS+=-Wl,-T,stm32l15xxb_flash.ld

PLATFORM=stm32l1xx
LIBS_STM_PATH=../libs_stm

# stm32l_discovery lib
CFLAGS+=-I$(LIBS_STM_PATH)/inc/base
CFLAGS+=-I$(LIBS_STM_PATH)/inc/core_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/device_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/$(PLATFORM)

LDFLAGS+=-L$(LIBS_STM_PATH)/build -lstm32_stdperiph_l1xx
LDFLAGS+=-L../startup_files -lstm32_stdperiph_l1xx
LDFLAGS+=-Wl,-Map=$(TARGET).map

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c ../startup_files/cm3_genstartup.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

write: all
	../../flash/flash write $(BIN_IMAGE) 0x08000000

.PHONY: all clean write
