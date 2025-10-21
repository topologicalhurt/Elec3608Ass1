#  NERV -- Naive Educational RISC-V Processor
#
#  Copyright (C) 2020  N. Engelhardt <nak@yosyshq.com>
#  Copyright (C) 2020  Claire Xenia Wolf <claire@yosyshq.com>
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

TOOLCHAIN_PREFIX?=riscv64-unknown-elf-
VERILATOR = verilator

# MacOS/Windows w/ docker desktop
#RDOCKER = docker run --platform linux/amd64 -it -e DISPLAY=host.docker.internal:0 -v `pwd`:/config phwl/elec3608-cad:latest

# Linux w/o SNAP & docker desktop
RDOCKER = docker run --platform linux/amd64 -it --net host -e DISPLAY=$(DISPLAY) -v `pwd`:/config phwl/elec3608-cad:latest

# this can be simply overridden using make FIRMWAREFILE=... all
FIRMWAREFILE=tests/test1-hart1-only.hex

# logging file name
LOGFILE=tests/`basename $(FIRMWAREFILE) .hex`.log
ASMFILE=tests/`basename $(FIRMWAREFILE) .hex`.s 
RESULTFILE=tests/results.log

# all tests
ALLFIRMWARE=tests/test1-hart1-only.hex tests/test2-hart2-only.hex tests/test3-samecode-bothharts.hex \
	tests/test4-race-hart1-slowslow.hex tests/test5-race-hart2-slowslow.hex \
	tests/test6-race-hart1-slow.hex  tests/test7-race-hart2-slow.hex  tests/test8-samecode-2reserv.hex \
	tests/test9-samecode-1reserv.hex \
	tests/test10-hart1-inside-hart2.hex  tests/test11-hart1-interleave-hart2.hex \
	tests/test12-hart1-interleave-hart2.hex  tests/test13-hart2-interleave-hart1.hex

all:	$(FIRMWAREFILE) test_verilator

allresults:
	$(foreach ff, $(ALLFIRMWARE),  make result FIRMWAREFILE=$(ff);)
	@echo "---------------------------------------------------"
	@echo "Results..."
	@cat $(RESULTFILE)

result : all 
	@python tests/checkregs.py $(LOGFILE) $(ASMFILE) | tee --append $(RESULTFILE)

test_verilator: testbench_verilator 
	@./obj_dir/Vtestbench +vcd > $(LOGFILE)

testbench_verilator: $(FIRMWAREFILE) tests/testbench-2hart.sv nerv-atomic.sv tests/testbench.cpp 
	@$(VERILATOR) --cc --exe -Wno-lint -trace --top-module testbench \
		-Gfirmwarefile=\"$(FIRMWAREFILE)\" \
		tests/testbench-2hart.sv nerv-atomic.sv tests/testbench.cpp
	@$(MAKE) -C obj_dir -f Vtestbench.mk

%.elf: %.s 
	@$(TOOLCHAIN_PREFIX)gcc -march=rv32ia -mabi=ilp32 -Os -Wall -Wextra -Wl,-Bstatic,-T,sections.lds,--strip-debug -ffreestanding -nostdlib -o $@ $^

%.hex: %.elf
	@$(TOOLCHAIN_PREFIX)objcopy -O verilog $< $@

show:
	@gtkwave testbench.vcd testbench.gtkw >> gtkwave.log 2>&1 

rundocker:
	$(RDOCKER)

clean:
	rm -rf firmware.elf firmware.hex testbench testbench.vcd gtkwave.log
	rm -rf disasm.o disasm.s checks/ cexdata/ obj_dir
	rm -rf tests/*.log tests/*.hex tests/*.elf
	rm -rf *.log *.asc *.json *.hex
