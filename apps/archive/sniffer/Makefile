#!/bin/sh

# define the root of the Contiki source tree (relative to this directory), the application name 
# and the system-wide Contiki makefile, which contains the definitions of the Contiki core system 
# and points out to the specific Makefile of our target platform. Makefile.include must always be 
# located in the root folder of the Contiki source tree.
CONTIKI = ../..
CONTIKI_PROJECT = sniffer

SRCS = ${shell find -L . -maxdepth 1 -type f -name "*.[c]" -printf "%f "}

target ?= dpp2

#export LD_LIBRARY_PATH3 = "$(HOME)/ti/ccsv7/ccs_base/DebugServer/drivers/"

include $(CONTIKI)/Makefile.include



