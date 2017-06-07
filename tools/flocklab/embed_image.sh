#!/bin/bash

if [ "$1" == "" ]; then
  echo "$0: no file name provided\n\nusage: $0 [COM image filename] [APP image filename (optional)]"
  exit
fi

cat flocklab-dpp-template.xml > flocklab-dpp.xml
  
# 1st image is for COM
if [ -f "$1" ]; then
  base64 $1 > $1.b64
  sed -i "/<\!-- COM IMAGE --><data>/r $1.b64" flocklab-dpp.xml
  rm $1.b64
  echo "COM binary image '$1' embedded into flocklab-dpp.xml"
else 
  echo "file $1 not found"
fi

# 2nd image is for APP
if [ "$2" != "" ]; then
  if [ -f "$2" ]; then
    if [ "$2" == *".b64" ]; then
      # file format is already base64
      echo "b64"
      cat $2 > $2.tmp
    elif [[ "$2" == *".hex" ]] || [[ "$2" == *".ihex" ]]; then
      # intel hex format
      msp430-objcopy -F elf32-msp430 $2 $2.tmp2
      base64 $2.tmp2 > $2.tmp
      rm $2.tmp2
      echo "hex"
    else
      # executable (.exe, .out)
      base64 $2 > $2.tmp
      echo "exe"
    fi
    sed -i "/<\!-- APP IMAGE --><data>/r $2.tmp" flocklab-dpp.xml
    rm $2.tmp
    echo "APP binary image '$2' embedded into flocklab-dpp.xml"
  else
    echo "file $2 not found"
  fi
fi
