###
# Analyzes the serial output of FlockLab
# the script searches the last print out of each node/observer and 
# calculates the average of 'hop=' values for each node
###

import sys
import re

filename = "serial.csv"

results = {}
hops = {}

def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def split_output(s):
    rcvd = 0
    miss = 0
    boot = 0
    per = 0
    m_hop = 0
    try:
        #idx = l[4].index("rcv=") + 4
        #rcvd = int(l[4][idx:].split(" ",1)[0])
        match = re.search(r"rcv=\d+", node)
        if match:
            rcvd = int(match.group()[4:])
        match = re.search(r"miss=\d+", output)
        if match:
            miss = int(match.group()[5:])
        match = re.search(r"boot=\d+", output)
        if match:
            boot = int(match.group()[5:])
        match = re.search(r"per=\d+", output)
        if match:
            per = int(match.group()[4:])
        match = re.search(r"m_hop=\d+", output)
        if match:
            m_hop = int(match.group()[6:])
    except ValueError:
        pass
        

# --- program execution starts here ---

# the first argument (if any) is the name of the subfolder
if len(sys.argv) > 1:
    filename = sys.argv[1] + "/" + filename
        
# loop through all lines
try:
    # option: lines = [line.rstrip('\n') for line in open('serial.csv')]
    with open(filename) as f:
        print "analyzing file '%s', please wait..." % filename    
        for line in f:
            l = line.strip('\n').split(",", 4)
            output = l[4]
            node_id = l[1]
            if is_int(node_id):
                node_id = ('00' + node_id)[-3:]
                # just overwrite, assume entries are sorted by timestamp
                results[node_id] = output  
                # extract the number of hops
                match = re.search(r"hop=\d+", output)
                if match:
                    try:
                        hops[node_id] = [hops[node_id][0] + int(match.group()[4:]), hops[node_id][1] + 1]
                    except KeyError:
                        hops[node_id] = [int(match.group()[4:]), 1]
except IOError:
    print "file '%s' not found" % filename
    sys.exit()
            
print "%d node(s) found" % len(results)
        
for node_id in sorted(results):
    try:
        print node_id + ": " + results[node_id][results[node_id].index("rcv="):] + " avg_hops=%.1f" % (float(hops[node_id][0]) / float(hops[node_id][1]))
    except ValueError:
        print "INVALID entry for node %d: '%s'" % (int(node_id), results[node_id])
        pass
    