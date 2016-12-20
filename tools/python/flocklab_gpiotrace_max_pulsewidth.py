#!/usr/bin/env python

'''
2016, Reto Da Forno

calculates the max. of all pulses of a signal in the gpio tracing output of flocklab

'''

import sys
import os


time_offset = 1.0           # skip the first x seconds
signal_name = "INT1"        # signal of interest


def read_csv_file(filename):
    count = 0
    diff_max = {}
    time_max = {}
    test_start = 0
    
    try:
        starttimes = {}
        print "loading data, please wait..."
        
        for line in open(filename):
            count = count + 1
            if count is 1:
                continue                            # skip the first line
            [timestamp, obs_id, node_id, pin_name, value] = line.split(',')
            if test_start is 0:
                test_start = int(float(timestamp))  # floor to seconds
                print "start set to " + str(test_start)
            if (float(timestamp) - test_start) < time_offset:
                continue
            if obs_id == 0:
                print "invalid line"
                break
            if signal_name not in pin_name:         # signal name must match
                continue
            if node_id not in starttimes:
                starttimes[node_id] = 0             # init
            if int(value) is 1:                     # rising edge
                if starttimes[node_id] is not 0:
                    print "unexpected transition on line " + str(count)
                starttimes[node_id] = float(timestamp)
            else:                            # falling edge
                if starttimes[node_id] is not 0:
                    diff = float(timestamp) - starttimes[node_id]
                    if node_id not in diff_max:
                        diff_max[node_id] = 0       # init
                    if diff_max[node_id] < diff:
                        diff_max[node_id] = diff
                        time_max[node_id] = starttimes[node_id] - test_start
                    starttimes[node_id] = 0         # reset
                else:
                    print "start found"
                    
            #if count > 100000:
            #    break
            
        print "data processed (" + str(count) + " rows)"
                
    except IOError as e:
        print "I/O error: " + e.strerror
        return False
    except IndexError:
        print "index error on line " + str(count) + " (not enough columns found), please check the CSV file"
        return False
    except ValueError:
        print "invalid data found on line " + str(count) + ", please check the CSV file"
        return False
        
    # print the max
    for node in diff_max:
        print "max of " + node + " is " + str(diff_max[node]) + " found at " + str(time_max[node])
    

# END of read_csv_file()


        
# main
if len(sys.argv) > 1:
    if os.path.isfile(sys.argv[1]):
        read_csv_file(sys.argv[1])
    else:
        print "file '" + sys.argv[1] + "' not found"
else:
    print "\nusage:\n\t" + os.path.basename(__file__) + " [filename].csv\n"
