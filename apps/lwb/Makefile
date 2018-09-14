#!/bin/sh
CONTIKI = ../..
CONTIKI_PROJECT = lwb-test
DESCRIPTION ?= LWB Test app

SRCS = ${shell find . -maxdepth 1 -type f -name "*.[c]" -printf "%f "}

TARGET ?= dpp

TESTDURATION ?= 120

FLOCKLABTOOLS = ./
XMLTEMPLATE = flocklab-$(TARGET)-template.xml
XMLFILE = flocklab-$(TARGET).xml

include $(CONTIKI)/Makefile.include


flocklab_xml:
	@base64 $(CONTIKI_PROJECT).exe > $(CONTIKI_PROJECT).$(TARGET).b64
	@sed -n '1h;1!H;$${ g;s/<data>[^<]*<\/data>/<data_new>\n<\/data>/;p}' $(FLOCKLABTOOLS)$(XMLTEMPLATE) > $(XMLFILE)
	@sed -i '/<data_new>/r $(CONTIKI_PROJECT).$(TARGET).b64' $(XMLFILE)
	@sed -i 's/<data_new>/<data>/' $(XMLFILE)
	@sed -i 's/<name>Name<\/name>/<name>$(CONTIKI_PROJECT)<\/name>/' $(XMLFILE)
	@sed -i 's/<description>Description<\/description>/<description>$(DESCRIPTION) \($(TARGET)\)<\/description>/' $(XMLFILE)
	@sed -i 's/<durationSecs>[^<]*<\/durationSecs>/<durationSecs>$(TESTDURATION)<\/durationSecs>/' $(XMLFILE)
	@rm $(CONTIKI_PROJECT).$(TARGET).b64

flocklab_test: flocklab_xml
	flocklab -c $(XMLFILE)


