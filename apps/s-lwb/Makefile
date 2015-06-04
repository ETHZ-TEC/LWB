
CONTIKI_PROJECT = s-lwb

SRCS = s-lwb.c scheduler.c compress.c mm.c stream.c

WITH_GLOSSY = 1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include


all: $(CONTIKI_PROJECT)

flocklab: 
# embed image
	@base64 $(CONTIKI_PROJECT).exe > $(CONTIKI_PROJECT).$(TARGET).b64
	@sed -i -n '1h;1!H;$${ g;s/<data>.*<\/data>/<data>\n<\/data>/;p}' flocklab-cc430.xml
	@sed -i '/<data>/r $(CONTIKI_PROJECT).$(TARGET).b64' flocklab-cc430.xml
	@rm $(CONTIKI_PROJECT).$(TARGET).b64

upload: 
	@export LD_LIBRARY_PATH=/opt/ti/ccsv6/ccs_base/DebugServer/drivers/
	@mspdebug tilib "prog $(CONTIKI_PROJECT).hex"

