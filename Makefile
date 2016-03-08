#!/bin/sh

CONTIKI = .
CONTIKI_PROJECT = glossy-test
APPDIR = ./apps/glossy

SRCS = ${shell find $(APPDIR) -maxdepth 1 -type f -name "*.[c]" -printf "%f "}

TARGET=olimex

include $(CONTIKI)/Makefile.include

flocklab:
# embed image
	@base64 $(CONTIKI_PROJECT).exe > $(CONTIKI_PROJECT).$(TARGET).b64
	@sed -i -n '1h;1!H;$${ g;s/<data>.*<\/data>/<data>\n<\/data>/;p}' flocklab-cc430.xml
	@sed -i '/<data>/r $(CONTIKI_PROJECT).$(TARGET).b64' flocklab-cc430.xml
	@rm $(CONTIKI_PROJECT).$(TARGET).b64
