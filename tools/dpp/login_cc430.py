#!/usr/bin/env python3
'''

Print the serial output of the CC430 MCU.

last update: 2018-09-24
author:      rdaforno

'''

import serial
import sys
import time
import os.path
import serial
import serial.tools.list_ports
from serial.serialutil import SerialException


baudRate = 115200


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
    ser = serial.Serial(port=serialPort, baudrate=baudRate)
    if ser.is_open:
      ser.close()
      return True
  except:
    print("can't connect to serial port " + serialPort)
  return False


def serial_read(serialPort):
  try:
    ser = serial.Serial(port=serialPort, baudrate=baudRate, timeout=None)
    if ser.isOpen():
      print "connected to " + ser.portstr + " (" + str(ser.baudrate) + ")"
      ser.setRTS(True)          # pull TEST / BSL entry line
      ser.setDTR(False)         # release reset line
      ser.flushInput()          # flush input and output buffers
      ser.flushOutput()
      while True:
        if ser.inWaiting() > 0:
          line = ser.read(ser.inWaiting())
          sys.stdout.write(line)
          sys.stdout.flush()
        time.sleep(0.01) 
  except SerialException:
      print "device %s unavailable" % serialPort
  except ValueError:
      print "invalid arguments"
  except OSError:
      print "device %s not found" % serialPort
  except KeyboardInterrupt:
      print "aborted"
  except:
      type, value, tb = sys.exc_info()
      print "error: %s (%s)" % (value.message, type)
  if ser.isOpen():
    ser.close()


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("no filename provided\r\nusage:  ./" + os.path.basename(__file__) + " [filename] [port (optional)]")
    sys.exit()
  fileName = sys.argv[1]
  if not os.path.isfile(fileName):
    print("file '%s' not found" % fileName)
    sys.exit()
  if len(sys.argv) > 2 and len(sys.argv[2]) > 3:
    # 2nd argument is supposed to be the serial port
    serialPort = sys.argv[2]
  else:
      serialPort = getFirstPort(False)
      if serialPort is None:
        print("no DPP2 DevBoard found")
        sys.exit()
  if not checkSerialPort(serialPort):
    sys.exit()
  serial_read(serialPort)
