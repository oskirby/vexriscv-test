
firmware:
	make -C firmware

ice40: firmware
	make -f Makefile.ice40

ecp5: firmware
	make -f Makefile.ecp5

sim: firmware
	make -f Makefile.sim

clean:
	make -C firmware clean
	make -f Makefile.ecp5 clean
	make -f Makefile.ice40 clean
	make -f Makefile.sim clean

.PHONY: ice40 ecp5 sim firmware clean
