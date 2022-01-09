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
# Makefile for Simulation and Synthesis
#------------------------------------------------------------------------------

# Paths
export ROOT_DIR := $(shell pwd)


# Targets
.PHONY: clean rtl gate synth help

default: clean rtl

rtl: 
	cd tb && $(MAKE) rtl

gate: synth
	cd tb && $(MAKE) gate

synth:
	cd ./synth && $(MAKE) synth

help:
	@echo "To run RTL  simulation: make rtl"
	@echo "To run Gate simulation: make gate"
	@echo "To run synthesis: make synth"



clean:
	cd tb && $(MAKE) clean && cd ../synth && $(MAKE) clean 
