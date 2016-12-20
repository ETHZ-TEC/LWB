#!/usr/bin/env python

# converts the unix timestamps in the serial.csv file (Flocklab serial output) into a more human readable format
# 
# usage:
# ./convert_timestamp.py [test_no]

import sys
import datetime

if len(sys.argv) > 1:
    filename = sys.argv[1] + "/serial.csv"
else:
	filename = "serial.csv"

with open(filename) as f:
    content = f.readlines()

output = ""
for line in content:
	parts = line.split('.', 1)
	if len(parts) != 2:
		continue
	try:
		sec = int(parts[0])
	except ValueError:
		print "unexpected conversion error (invalid file format)"
		sys.exit()
	[usec, remainder] = parts[1].split(',', 1)
	output += datetime.datetime.fromtimestamp(sec).strftime('%Y-%m-%d %H:%M:%S  ') + usec + 'us ' + remainder
	
if len(output) is not 0:
	with open(filename, 'w') as f:
		f.write(output)
