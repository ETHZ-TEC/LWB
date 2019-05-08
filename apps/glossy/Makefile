#!/bin/sh
CONTIKI = ../..
CONTIKI_PROJECT = glossy-test
DESCRIPTION ?= Glossy Test

SRCS = ${shell find . -maxdepth 1 -type f -name "*.[c]" -printf "%f "}

TARGET ?= dpp

TESTDURATION ?= 120

FLOCKLABTOOLS = ./
XMLTEMPLATE = flocklab-$(TARGET)-template.xml
XMLFILE = flocklab-$(TARGET).xml

include $(CONTIKI)/Makefile.include
include $(CONTIKI)/tools/flocklab/Makefile.flocklab
