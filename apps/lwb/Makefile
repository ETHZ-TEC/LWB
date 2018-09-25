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
include $(CONTIKI)/tools/flocklab/Makefile.flocklab

