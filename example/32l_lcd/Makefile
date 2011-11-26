ELF=lcd.elf

CC=arm-none-eabi-gcc
DEBUG?=1

CFLAGS=-O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m3 -ffreestanding -nostdlib -nostdinc
ifeq ($(DEBUG),1)
	CFLAGS+=-g
endif

CFLAGS+=-I.


PLATFORM=stm32l1xx
LIBS_STM_PATH=../libs_stm

CFLAGS+=-I$(LIBS_STM_PATH)/inc/base
CFLAGS+=-I$(LIBS_STM_PATH)/inc/core_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/device_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/$(PLATFORM)

# Boot from flash, known to work, by default...
BOOT_FLASH?=1
ifeq ($(BOOT_FLASH),1)
	CFLAGS+=-Wl,-T,stm32_flash.ld
else
	CFLAGS+=-Wl,-T,linker_stm32l.lds
	CFLAGS+=-DCONFIG_BOOT_SRAM=1
endif

LDFLAGS+=-L$(LIBS_STM_PATH)/build -lstm32_stdperiph_l1xx

SRCS=\
main.c\
stm32l_discovery_lcd.c\
system_stm32l1xx.c\
startup_stm32l1xx_md.s

all: $(ELF)

$(ELF): $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	-rm -f $(ELF)

.PHONY: all clean
