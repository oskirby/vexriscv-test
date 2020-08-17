
CROSS_COMPILE ?= riscv64-unknown-elf-
CFLAGS = -march=rv32i -mabi=ilp32
LDFLAGS = $(CFLAGS) -Wl,-Bstatic,-T,firmware.lds -nostdlib

OBJS = start.o main.o

# Pattern Rules
%.o: %.s
	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $<

%.o: %.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $<

%.bin: %.elf
	$(CROSS_COMPILE)objcopy -O binary $^ $@

%.mem: %.bin
	hexdump  -e '"" 1/4 "%08x\n"' $^ > $@

# Firmware Build Rules.
firmware.elf: $(OBJS)
	$(CROSS_COMPILE)gcc $(LDFLAGS) -o $@ $^

.DEFAULT_GOAL = all
.PHONY: all clean

all: firmware.mem

clean:
	rm -f *.o *.elf *.bin *.mem
