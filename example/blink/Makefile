CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

DEF_CFLAGS=-g -O2 -mlittle-endian -mthumb -ffreestanding -nostdlib -nostdinc
DEF_CFLAGS+=-I../libs_stm/inc/base
DEF_CFLAGS+=-I../libs_stm/inc/core_support
DEF_CFLAGS+=-I../libs_stm/inc/device_support
DEF_CFLAGS+=-L../ld_scripts

# to run from SRAM
#DEF_CFLAGS+=-Wl,-Ttext,0x20000000 -Wl,-e,0x20000000

CFLAGS_VL=$(DEF_CFLAGS) -mcpu=cortex-m3 -DCONFIG_STM32VL_DISCOVERY=1 -DSTM32F10X_MD_VL
CFLAGS_L=$(DEF_CFLAGS) -mcpu=cortex-m3 -DCONFIG_STM32L_DISCOVERY
CFLAGS_F4=$(DEF_CFLAGS) -mcpu=cortex-m4 -DCONFIG_STM32F4_DISCOVERY=1

all: blink_32VL.bin blink_32L.bin blink_F4.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@
SRCS=main.c cm3_genstartup.c

blink_32VL.elf: $(SRCS)
	$(CC) $(CFLAGS_VL) -Wl,-T,stm32f100xb_flash.ld $^ -o $@
blink_32L.elf: $(SRCS)
	$(CC) $(CFLAGS_L) -Wl,-T,stm32l15xxb_flash.ld $^ -o $@
blink_F4.elf: $(SRCS)
	$(CC) $(CFLAGS_F4) -Wl,-T,stm32f4xxg_flash.ld $^ -o $@

clean:
	rm -rf *.elf
	rm -rf *.bin

.PHONY: all clean
