# //////////////////////////////////////////////////////////////////////////////
# // SPDX-FileCopyrightText: 2021, Dinesh Annayya
# // 
# // Licensed under the Apache License, Version 2.0 (the "License");
# // you may not use this file except in compliance with the License.
# // You may obtain a copy of the License at
# //
# //      http://www.apache.org/licenses/LICENSE-2.0
# //
# // Unless required by applicable law or agreed to in writing, software
# // distributed under the License is distributed on an "AS IS" BASIS,
# // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# // See the License for the specific language governing permissions and
# // limitations under the License.
# // SPDX-License-Identifier: Apache-2.0
# // SPDX-FileContributor: Dinesh Annayya <dinesha@opencores.org>
# // //////////////////////////////////////////////////////////////////////////
#------------------------------------------------------------------------------
# Makefile for Synthesis
#------------------------------------------------------------------------------

# Paths
export ROOT_DIR := $(shell pwd)

### To Enable IVERILOG FST DUMP
export IVERILOG_DUMPER = fst


# Targets
.PHONY: clean rtl gate help

default: clean rtl

rtl: clean  
	iverilog -g2012  -D SPEEDSIM  -DFUNCTIONAL -DSIM -DWFDUMP -I ../src -I ../lib -I ../model -I $(PDK_ROOT)/sky130A  tb_top.v  -o tb_top.vvp
	vvp tb_top.vvp

gate: clean  
	iverilog -g2012  -D GL -D SPEEDSIM  -DFUNCTIONAL -DSIM -DWFDUMP -I ../verilog/gl/  -I ../model -I $(PDK_ROOT)/sky130A  tb_top.v  -o tb_top.vvp
	vvp tb_top.vvp

help:
	@echo "To run RTL  simulation: make rtl"
	@echo "To run Gate simulation: make gate"



clean:
	$(RM) -R *.vvp
	$(RM) -R *.vcd
	$(RM) -R *.fst
