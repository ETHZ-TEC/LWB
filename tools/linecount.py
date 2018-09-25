#!/usr/bin/python
#
# 2018 by rdaforno
#
# counts the lines of code, excludes comments and blank lines
#
# either pass a list of files or a directory to this script
# to get a list of files within a folder, use e.g.:
#   find [dir] -type f -name "*.[c|h]" -printf "[dir]/%f "

import sys
import re
import os
from glob import glob

debug = False

if len(sys.argv) < 2:
  print "not enough arguments provided\r\nusage:\t" + sys.argv[0] + " [dir or file1] [file2] ..."
  sys.exit()

filelist = sys.argv[1:]
if os.path.isdir(sys.argv[1]):
  filelist = [y for x in os.walk(sys.argv[1]) for y in glob(os.path.join(x[0], '*.c'))]
  filelist += [y for x in os.walk(sys.argv[1]) for y in glob(os.path.join(x[0], '*.h'))]
  print " ".join(filelist)

commentBegin = re.compile(r"\/\*.*")
commentEnd = re.compile(r".*\*\/")
commentOneline = re.compile(r"\/\*.*\*\/|\/\/.*$")

linecnt = 0
filecnt = 0
output  = ""
skip    = False

for filename in filelist:
  if not os.path.isfile(filename):
    print "file '" + filename + "' skipped"
    continue
  linecntfile = 0
  content = [line for line in open(filename, 'r')]
  for line in content:
    if skip:
      # look for end of multiline comment
      if re.search(commentEnd, line) is not None:
        line = re.sub(commentEnd, "", line)
        skip = False
      else:
        continue
    else:
      # remove single line comments
      line = re.sub(commentOneline, "", line)
      # look for begin of multiline comments
      if re.search(commentBegin, line) is not None:
        line = re.sub(commentBegin, "", line)
        skip = True
    # remove all whitespaces from the right side
    line = line.rstrip()
    # count as a line of code if there are still characters remaining
    if len(line) > 0:
      linecntfile = linecntfile + 1
      if debug:
        output += line + '\n'

  if debug:
    print output
  print "lines of code in '" + sys.argv[1] + "': " + str(linecntfile)
  linecnt = linecnt + linecntfile
  filecnt = filecnt + 1

print "total lines of code in all " + str(filecnt) + " files: " + sys.argv[1] + "': " + str(linecnt)

