#!/bin/sh

# define the root of the Contiki source tree (relative to this directory), the application name 
# and the system-wide Contiki makefile, which contains the definitions of the Contiki core system 
# and points out to the specific Makefile of our target platform. Makefile.include must always be 
# located in the root folder of the Contiki source tree.
CONTIKI = ../..
CONTIKI_PROJECT = elwb-dev

SRCS = ${shell find . -maxdepth 1 -type f -name "*.[c]" -printf "%f "}

node_id ?= 0x1122
target ?= dpp2

include $(CONTIKI)/Makefile.include

upload:
	tos-set-symbols --objcopy msp430-objcopy --objdump msp430-objdump --target ihex $(CONTIKI_PROJECT).exe $(CONTIKI_PROJECT).hex TOS_NODE_ID=$(node_id)
	@export LD_LIBRARY_PATH="/home/keepcoding/ti/ccsv7/ccs_base/DebugServer/drivers/"
	@mspdebug tilib "prog $(CONTIKI_PROJECT).hex" --allow-fw-update
