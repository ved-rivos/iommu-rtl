#===================================================================
#
# Makefile
# --------
# Makefile for building the iommu core and top simulations.
#
#
# ved@rivosinc.com
# Copyright (c) 2022, Rivos inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or
# without modification, are permitted provided that the following
# conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#===================================================================

CORE_SRC=./src/rtl/iommu_mmio.sv ./src/rtl/iommu_walker.sv ./src/rtl/iommu_ddtc.sv ./src/rtl/iommu_pdtc.sv ./src/rtl/iommu_lspa.sv

TOP_SRC=./src/rtl/iommu.sv $(CORE_SRC)
TB_TOP_SRC=./src/tb/tb_iommu.sv

CC = iverilog -g2012 -gstrict-expr-width -I src/rtl/
CC_FLAGS = -Wall  

LINT = verilator 
LINT_FLAGS = +1364-2001ext+ +incdir+./src/rtl/ --lint-only  -Wall -Wno-fatal -Wno-DECLFILENAME 


all: top.sim core.sim


top.sim: $(TB_TOP_SRC) $(TOP_SRC)
	$(CC) $(CC_FLAGS) -o top.sim $(TB_TOP_SRC) $(TOP_SRC)


core.sim: $(TB_CORE_SRC) $(CORE_SRC)
	$(CC) $(CC_FLAGS) -o core.sim $(TB_CORE_SRC) $(CORE_SRC)


sim-top: top.sim
	./top.sim


sim-core: core.sim
	./core.sim


lint:  $(TOP_SRC)
	$(LINT) $(LINT_FLAGS) $(TOP_SRC)


clean:
	rm -f top.sim
	rm -f core.sim
	rm -f *.vcd


help:
	@echo "Build system for simulation of Prince core"
	@echo ""
	@echo "Supported targets:"
	@echo "------------------"
	@echo "all:          Build all simulation targets."
	@echo "top.sim:      Build top level simulation target."
	@echo "core.sim:     Build core level simulation target."
	@echo "sim-top:      Run top level simulation."
	@echo "sim-core:     Run core level simulation."
	@echo "lint:         Lint all rtl source files."
	@echo "clean:        Delete all built files."

#===================================================================
# EOF Makefile
#===================================================================
