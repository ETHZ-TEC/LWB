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


def listAvailablePorts():
  print("available serial ports:")
  ports = [p for p in serial.tools.list_ports.comports()]
  for p in sorted(ports):
    print("%s" % p)


def getDPPDevBoardPort(printPorts):
  ports = [p for p in serial.tools.list_ports.comports() if "Dual RS232" in p[1]]
  if printPorts:
    for p in sorted(ports):
      print("%s" % p)
  if ports is None or len(ports) == 0:
    return None
  return sorted(ports)[1][0]


def getMSPFETPort(printPorts):
  ports = [p for p in serial.tools.list_ports.comports() if "MSP Debug" in p[1]]
  if printPorts:
    for p in sorted(ports):
      print("%s" % p)
  if ports is None or len(ports) == 0:
    return None
  return sorted(ports)[0][0]


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


def programWithMSPFET(serialPort, fileName):
  if "LD_LIBRARY_PATH" in os.environ:
    cmd = "mspdebug tilib 'prog " + fileName + "' --allow-fw-update"
    if serialPort:
      cmd += " -d " + serialPort
    ret = subprocess.call(cmd, shell=True)
    if ret == 0:
      return True
    print("failed")
  else:
    print("can't use MSP-FET, LD_LIBRARY_PATH not defined")
  return False


def programWithBSL(serialPort, fileName):
  print("connecting to serial port %s" % serialPort)
  ret = subprocess.call(['python', '-m', 'msp430.bsl5.uart', '-p', serialPort, '--invert-reset', '-e', '-S', '-s', '115200' , '-P', fileName])
  if ret == 0:
    return True
  print("failed")
  return False


if __name__ == "__main__":
  # check arguments
  if len(sys.argv) < 2:
    print("no filename provided\r\nusage:  ./" + os.path.basename(__file__) + " [filename] [port (optional)]")
    sys.exit()
  fileName = sys.argv[1]
  if not os.path.isfile(fileName):
    print("file '%s' not found" % fileName)
    sys.exit()

  # determine the serial port
  if len(sys.argv) > 2:
    # 2nd argument is supposed to be the serial port
    serialPort = sys.argv[2]
  else:
    # prefer the MSP-FET
    serialPort = getMSPFETPort(False)
    if serialPort is None:
      # no MSP-FET found, try to find a DPP DevBoard
      serialPort = getDPPDevBoardPort(False)
      if serialPort is None:
        print("no connected device found")
        listAvailablePorts()
        sys.exit()  # abort

  # do the target programming
  if "USB" in serialPort:
    if checkSerialPort(serialPort):
      if not programWithBSL(serialPort, fileName):
        listAvailablePorts()
      # reset the target
      if resetAfterProgram:
        resetMCU(serialPort)
  else:
    if not programWithMSPFET(serialPort, fileName):
      listAvailablePorts()
