##
## Make and program TinyFPGA BX
##
BASENAME = vextest
#BASENAME = picotest
PROJTOP = $(BASENAME)_lbone
SIMTOP = $(BASENAME)_sim

SOURCES = \
	lib/VexRiscv_Min.v \
	lib/addrdecode.v \
	lib/picorv32.v \
	lib/skidbuffer.v \
	lib/wbc2pipeline.v \
	lib/wbcdecoder.v \
	lib/wbcxbar.v \
	lib/wbxbar.v \
	wbleds.v \
	wbsram.v \
	bootrom.v

SRC = $(PROJTOP).v logicbone_pll.v $(SOURCES)

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

dfu: $(PROJTOP).bit
	dfu-util -d 1d50:615d -a0 -D $<

# Firmware Build Rules.
.PHONY: firmware/firmware.bin
firmware/firmware.bin:
	make -C firmware firmware.bin

firmware.mem: firmware/firmware.bin
	hexdump  -e '"" 1/4 "%08x\n"' $^ > $@

# Simulation Build Rules.
$(SIMTOP): $(SIMTOP).v $(SOURCES) $(SIMTOP).cpp firmware.mem
	verilator --trace --top-module $(SIMTOP) -cc $(filter %.v,$^) -Wno-fatal --exe $(SIMTOP).cpp
	make -C obj_dir -f V$(SIMTOP).mk

sim: $(SIMTOP)

# Cleanup Rules
clean:
	make -C firmware clean
	rm -f $(PROJTOP).json $(PROJTOP).svf $(PROJTOP).bit $(PROJTOP)_out.config $(PROJTOP).hex
	rm -f firmware.mem
	rm -f $(SIMTOP) $(SIMTOP).vcd
	rm -rf obj_dir/

.SECONDARY:
.PHONY: all bitstream synth dfu clean gui
