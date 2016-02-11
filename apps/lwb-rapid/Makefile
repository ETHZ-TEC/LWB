#!/bin/sh

# define the root of the Contiki source tree (relative to this directory), the application name 
# and the system-wide Contiki makefile, which contains the definitions of the Contiki core system 
# and points out to the specific Makefile of our target platform. Makefile.include must always be 
# located in the root folder of the Contiki source tree.
CONTIKI = ../..
CONTIKI_PROJECT = lwb

# specify the target platform (or pass this parameter by command line)
#TARGET = cc430

SRCS = ${shell find . -type f -name "*.[c]" -printf "%f "}
#$(info hello world = $(SRCS))

include $(CONTIKI)/Makefile.include

all: $(CONTIKI_PROJECT)

flocklab:
# embed image
	@base64 $(CONTIKI_PROJECT).exe > $(CONTIKI_PROJECT).$(TARGET).b64
	@sed -i -n '1h;1!H;$${ g;s/<data>.*<\/data>/<data>\n<\/data>/;p}' flocklab-cc430.xml
	@sed -i '/<data>/r $(CONTIKI_PROJECT).$(TARGET).b64' flocklab-cc430.xml
	@rm $(CONTIKI_PROJECT).$(TARGET).b64

upload: 
	@export LD_LIBRARY_PATH="/home/$(USER)/ti/ccsv6/ccs_base/DebugServer/drivers/"
	@mspdebug tilib "prog $(CONTIKI_PROJECT).hex" --allow-fw-update

