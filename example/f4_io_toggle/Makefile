# build an elf from the an3268 demonstration code
APP=f4_io_toggle

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4 -ffreestanding -nostdlib -nostdinc
ifeq ($(DEBUG),1)
	CFLAGS+=-g
endif

# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

PLATFORM=stm32f4xx
BOARD_SUPPORT=../board_support/discovery_32f4
LIBS_STM_PATH=../../libs_stm

CFLAGS+=-I.
CFLAGS+=-I$(LIBS_STM_PATH)/inc/base
CFLAGS+=-I$(LIBS_STM_PATH)/inc/core_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/device_support
CFLAGS+=-I$(LIBS_STM_PATH)/inc/$(PLATFORM)
CFLAGS+=-I$(BOARD_SUPPORT)

LDFLAGS+=-L$(LIBS_STM_PATH)/build -lstm32_stdperiph_f4xx


SRCS=$(wildcard *.c)
SRCS+=$(wildcard $(BOARD_SUPPORT)/*.c)

OBJS=$(SRCS:.c=.o)

all: $(APP).elf

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

$(APP).elf: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

clean:
	rm -rf $(OBJS)
	rm -rf $(APP).*

.PHONY: all clean
