# Tool names
PREFIX=arm-none-eabi-
CC          := $(PREFIX)gcc
OBJCOPY     := $(PREFIX)objcopy
SIZE        := $(PREFIX)size

REVISION := $(shell git log -1 --format="%h")

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
LTO_FLAGS	 = -O2 -flto -fuse-linker-plugin -ffunction-sections -fdata-sections -fverbose-asm -ffat-lto-objects
WARN_FLAGS   	 = -Werror -Wfatal-errors -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef  -Wno-pedantic
DEBUG_FLAGS	 = -ggdb3 -DNDEBUG -D__REVISION__='"$(REVISION)"' 
CFLAGS 		 = -std=gnu99 $(ARCH_FLAGS) $(LTO_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS)
LDFLAGS		 = -nostartfiles -lnosys -static $(ARCH_FLAGS) $(LTO_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS) -Wl,-gc-sections,-Map,main.map -Wl,--cref

.DEFAULT_GOAL := main.hex

LD_SCRIPT = stm32_flash_f103_128k.ld

OBJS = \
	vectors.o \
	boot.o \
	clock.o \
	gpio2.o \
	serial.o \
	usb/st_usbfs.o \
	usb/usb_standard.o \
	usb/usb_control.o \
	usb/usb.o \
	main.o \

$(OBJS): Makefile

# Compile
%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

%.elf: $(LD_SCRIPT)
%.elf: $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS) -T$(LD_SCRIPT)

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex --set-start 0x08000000 $< $@

flash: main.hex
	st-flash --reset --format ihex write main.hex

debug: main.elf
	st-util &
	$(PREFIX)gdb $^
	killall st-util
	
clean:
	rm -f *~ *.o *.hex *.bin *.elf *.map

depend:
	makedepend -Y. -w150 *.c

# DO NOT DELETE

boot.o: stm32f103_md.h core_cm3.h
clock.o: stm32f103_md.h core_cm3.h clock.h
gpio2.o: gpio2.h stm32f103_md.h core_cm3.h
main.o: stm32f103_md.h core_cm3.h clock.h gpio2.h serial.h usb/usbd.h usb/usbstd.h
serial.o: serial.h stm32f103_md.h core_cm3.h stb_sprintf.h
vectors.o: stm32f103_md.h core_cm3.h
