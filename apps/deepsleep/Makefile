#!/bin/sh

# define the root of the Contiki source tree (relative to this directory), the application name 
# and the system-wide Contiki makefile, which contains the definitions of the Contiki core system 
# and points out to the specific Makefile of our target platform. Makefile.include must always be 
# located in the root folder of the Contiki source tree.
CONTIKI = ../..
CONTIKI_PROJECT = deepsleep

SRCS = ${shell find . -maxdepth 1 -type f -name "*.[c]" -printf "%f "}

include $(CONTIKI)/Makefile.include

flocklab_xml:
# embed image
	@base64 $(CONTIKI_PROJECT).exe > $(CONTIKI_PROJECT).b64
	@sed -i -n '1h;1!H;$${ g;s/<data>[^<]*<\/data>/<datacc430>\n<\/data>/;p}' flocklab-dpp.xml
	@sed '/<datacc430>/r $(CONTIKI_PROJECT).b64' -i flocklab-dpp.xml
	@sed -i 's/<datacc430>/<data>/' flocklab-dpp.xml
	@echo base64 image embedded into 'flocklab-dpp.xml'
	@rm $(CONTIKI_PROJECT).b64

flocklab: flocklab_xml
