CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

DEF_CFLAGS=-g -O2 -mlittle-endian -mthumb -ffreestanding -nostdlib -nostdinc
DEF_CFLAGS+=-I../libs_stm/inc/base
DEF_CFLAGS+=-I../libs_stm/inc/core_support
DEF_CFLAGS+=-I../libs_stm/inc/device_support
DEF_CFLAGS+=-L../startup_files

# to run from SRAM
# make BOOT_FLASH=0
BOOT_FLASH?=1
ifeq ($(BOOT_FLASH),1)
    LINK_TYPE=flash
else
    LINK_TYPE=sram
    CFLAGS+=-DCONFIG_BOOT_SRAM=1
    # This keeps the standard system_xxx files from doing any damage...
    CFLAGS+=-DVECT_TAB_SRAM
endif


CFLAGS_VL=$(DEF_CFLAGS) -mcpu=cortex-m3 -DCONFIG_STM32VL_DISCOVERY=1 -DSTM32F10X_MD_VL
CFLAGS_L=$(DEF_CFLAGS) -mcpu=cortex-m3 -DCONFIG_STM32L_DISCOVERY
CFLAGS_F4=$(DEF_CFLAGS) -mcpu=cortex-m4 -DCONFIG_STM32F4_DISCOVERY=1

all: blink_32VL.bin blink_32L.bin blink_F4.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@
SRCS=main.c ../startup_files/cm3_genstartup.c

blink_32VL.elf: $(SRCS)
	$(CC) $(CFLAGS_VL) -Wl,-T,stm32f100xb_$(LINK_TYPE).ld $^ -o $@
blink_32L.elf: $(SRCS)
	$(CC) $(CFLAGS_L) -Wl,-T,stm32l15xxb_$(LINK_TYPE).ld $^ -o $@
blink_F4.elf: $(SRCS)
	$(CC) $(CFLAGS_F4) -Wl,-T,stm32f4xxg_$(LINK_TYPE).ld $^ -o $@

clean:
	rm -rf *.elf
	rm -rf *.bin

.PHONY: all clean
