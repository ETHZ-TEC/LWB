#!/bin/bash

# flash the cc430 via serial cable (BSL)

FILENAME="elwb-dev.hex"
PORT="/dev/ttyUSB0"
DPPTOOLSPATH="../../../../../misc/tools/bsl/"

if [ "$#" -gt 0 ]; then
  PORT=$1
fi

python ${DPPTOOLSPATH}serialread.py $PORT --bsl-entry
sleep 1
python -m msp430.bsl5.uart -p $PORT --no-start -e -V -S -s 115200 -P $FILENAME
python ${DPPTOOLSPATH}serialread.py $PORT
