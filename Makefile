CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
LIBOPENCM3 ?= ./libopencm3

# Config bits
APP_ADDRESS = 0x08001000

PLATFORM_DEFS = -DSTM32F1 -mthumb -mcpu=cortex-m3

CFLAGS = -O2 -std=c11 -Wall -pedantic -Werror -Istm32/include \
	-ffunction-sections -fdata-sections -Wno-address-of-packed-member \
	-I$(LIBOPENCM3)/include -DAPP_ADDRESS=$(APP_ADDRESS)   \
	-ggdb -Icompression/ -Istm32-libs/include -Iecc  \
	-DVERSION=\"$(GIT_VERSION)\" -flto $(CONFIG_DEFINES) \
	$(PLATFORM_DEFS)

LDFLAGS = -lopencm3_stm32f1 -ggdb \
	-ffunction-sections -fdata-sections \
	-Wl,-Tstm32f103.ld -nostartfiles -lnosys \
	-L$(LIBOPENCM3)/lib/ -Wl,-gc-sections -flto \
	$(PLATFORM_DEFS)

all:	modem3.bin cli.bin

modem3.elf: main.o | $(LIBOPENCM3)/lib/libopencm3_stm32f1.a
	$(CC) $^ -o $@ $(LDFLAGS) -Wl,-Ttext=$(APP_ADDRESS) -Wl,-Map,modem3.map

$(LIBOPENCM3)/lib/libopencm3_stm32f1.a:
	$(MAKE) -C $(LIBOPENCM3) TARGETS=stm32/f1 AR=$(CC)-ar CFLAGS=-flto LDFLAGS=-flto

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@
	python3 stm32-bootloader/checksum.py $@

%.o: %.c $(LIBOPENCM3)/lib/libopencm3_stm32f1.a
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	-rm -f *.elf *.o *.bin *.map

cli.bin:	cli.cc
	g++ -o cli.bin cli.cc -lusb-1.0 -O2 -ggdb


