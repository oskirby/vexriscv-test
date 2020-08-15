##
## Make and program TinyFPGA BX
##
CROSS_COMPILE ?= riscv64-unknown-elf-

BASENAME = vextest
PROJTOP = $(BASENAME)_lbone
SIMTOP = $(BASENAME)_sim

SOURCES = \
	lib/VexRiscv_Min.v \
	lib/addrdecode.v \
	lib/picorv32.v \
	lib/skidbuffer.v \
	lib/wbxbar.v \
	wbleds.v \
	wbsram.v \
	bootrom.v

# Extra dependencies for hardware only.
$(PROJTOP).json: logicbone_pll.v

SRC = $(PROJTOP).v $(SOURCES)

PIN_DEF = logicbone-rev0.lpf

DEVICE = --um5g-85k
PACKAGE = CABGA381

all: $(PROJTOP).bit $(PROJTOP).hex $(PROJTOP).svf
bootloader: $(PROJTOP).bit

synth: $(PROJTOP).json

$(PROJTOP).json: $(SRC) firmware.mem
	yosys -q -p 'synth_ecp5 -top $(PROJTOP) -json $@' $(filter %.v,$^)

%_out.config: %.json
	nextpnr-ecp5 --json $< --textcfg $@ $(DEVICE) --package $(PACKAGE) --lpf $(PIN_DEF)

%.svf: %_out.config
	ecppack --svf $@ $<

%.bit: %_out.config
	ecppack --compress --spimode qspi $< $@

%.hex: %.bit
	hexdump $^ > $@

firmware.elf: firmware.s
	$(CROSS_COMPILE)gcc $(CFLAGS) -march=rv32i -mabi=ilp32 -Wl,-Bstatic,-T,firmware.lds -nostdlib -o $@ $^

firmware.bin: firmware.elf
	$(CROSS_COMPILE)objcopy -O binary $^ $@

firmware.mem: firmware.bin
	hexdump  -e '"" 1/4 "%08x\n"' $^ > $@

dfu: $(PROJTOP).bit
	dfu-util -d 1d50:615d -a0 -D $<

# Simulation Rules.
$(SIMTOP): $(SIMTOP).v $(SOURCES) firmware.mem
	verilator --trace --top-module $(SIMTOP) -cc $(filter %.v,$^) -Wno-fatal --exe $(SIMTOP).cpp
	make -C obj_dir -f V$(SIMTOP).mk

sim: $(SIMTOP)

clean:
	rm -f $(PROJTOP).json $(PROJTOP).svf $(PROJTOP).bit $(PROJTOP)_out.config $(PROJTOP).hex
	rm -f firmware.elf firmware.bin

.SECONDARY:
.PHONY: all bitstream synth dfu clean gui
