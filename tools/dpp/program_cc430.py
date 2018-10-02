#!/usr/bin/env python3
'''

Flash a hex file onto the CC430 MCU via the bootloader.

This routing only works for the DPP2 DevBoard.
The solder jumpers J502/J503 on the back of the DevBoard need to be closed.
The script will automatically pick the first available serial port that is
identified as an FTDI Dual RS232 device.

last update: 2018-07-24
author:      rdaforno

'''

import sys
import os.path
import serial
import serial.tools.list_ports
import subprocess
import time


resetAfterProgram = True


def getFirstPort(printPorts):
  ports = [p for p in serial.tools.list_ports.comports() if "Dual RS232" in p[1]]
  if printPorts:
    for p in sorted(ports):
      print("%s" % p)
  if ports is None or len(ports) == 0:
    return None
  return sorted(ports)[1][0]


def checkSerialPort(serialPort):
  try:
    ser = serial.Serial(port=serialPort, baudrate=115200)
    if ser.is_open:
      ser.close()
      return True
  except:
    print("can't connect to serial port " + serialPort)
  return False


def resetMCU(serialPort):
  try:
    ser = serial.Serial(port=serialPort, baudrate=115200, xonxoff=0, rtscts=0)
    if ser.is_open:
      ser.setDTR(True)        # pull reset line
      ser.setRTS(True)        # pull TEST / BSL entry line
      time.sleep(0.1)
      ser.setDTR(False)       # release reset line
      ser.close()
      print("target reset")
  except:
    print("failed to connect to serial port " + serialPort)


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("no filename provided\r\nusage:  ./" + os.path.basename(__file__) + " [filename] [port (optional)]")
    sys.exit()
  fileName = sys.argv[1]
  if not os.path.isfile(fileName):
    print("file '%s' not found" % fileName)
    sys.exit()
  if len(sys.argv) > 2:
    # 2nd argument is supposed to be the serial port
    serialPort = sys.argv[2]
  else:
      serialPort = getFirstPort(False)
      if serialPort is None:
        print("no DPP2 DevBoard found")
        sys.exit()
  if not checkSerialPort(serialPort):
    sys.exit()
  print("connecting to serial port %s" % serialPort)
  subprocess.call(['python', '-m', 'msp430.bsl5.uart', '-p', serialPort, '--invert-reset', '-e', '-S', '-s', '115200' , '-P', fileName])
  if resetAfterProgram:
    resetMCU(serialPort)

