#!/bin/sh

# define the root of the Contiki source tree (relative to this directory), the application name 
# and the system-wide Contiki makefile, which contains the definitions of the Contiki core system 
# and points out to the specific Makefile of our target platform. Makefile.include must always be 
# located in the root folder of the Contiki source tree.
#
# in order to use the upload command, the environment variable LD_LIBRARY_PATH must be defined 
# (e.g. export it in ~/.bashrc) and point to the tilib file libmsp430.so, which is included in 
# the TI Code Composer Studio or the MSPFlasher tool 

CONTIKI = .
CONTIKI_PROJECT = lwb-dev
APPDIR = ./apps/$(CONTIKI_PROJECT)

SRCS = ${shell find $(APPDIR) -maxdepth 1 -type f -name "*.[c]" -printf "%f "}

include $(CONTIKI)/Makefile.include

flocklab:
# embed image
	@base64 $(CONTIKI_PROJECT).exe > $(CONTIKI_PROJECT).b64
	@sed -i -n '1h;1!H;$${ g;s/<data>[^<]*<\/data>/<datacc430>\n<\/data>/;p}' flocklab-dpp.xml
	@sed '/<datacc430>/r $(CONTIKI_PROJECT).b64' -i flocklab-dpp.xml
	@sed -i 's/<datacc430>/<data>/' flocklab-dpp.xml
	@rm $(CONTIKI_PROJECT).b64

upload:
ifdef dev
	@mspdebug tilib "prog $(CONTIKI_PROJECT).hex" --allow-fw-update -d $(dev)
else
	@mspdebug tilib "prog $(CONTIKI_PROJECT).hex" --allow-fw-update
endif

