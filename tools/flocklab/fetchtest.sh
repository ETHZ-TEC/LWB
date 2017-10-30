#!/bin/bash
dav_site='https://www.flocklab.ethz.ch/user/webdav'
dav_client=/usr/bin/cadaver
scratch=/tmp/flocklab_tmp
maxtime=3600
fail=0
if [ $# -eq 1 ]
then
if [ ! -d $scratch ]
then
	mkdir $scratch
fi
if [ ! -d $scratch/$1 ]
then
	mkdir $scratch/$1
fi
mkdir /tmp/$1
starttime=`date +%s`
duration=$((`date +%s` - $starttime))
while [ $duration -lt $maxtime ]
do
$dav_client 1> $scratch/$1/dav.log 2> $scratch/$1/dav.err <<EOF
open $dav_site
get $1/testconfiguration.xml /tmp/$1/testconfiguration.xml
get $1/results_nopower.tar.gz $scratch/$1/results.tar.gz
exit
EOF
cat $scratch/$1/dav.log
cat $scratch/$1/dav.err
if [ `sed '/^\(404\|500\)/!d' $scratch/$1/dav.log | wc -l` -gt 0 ]
then
	echo "error occured while fetching data"
	fail=1
	sleep 120
	duration=$((`date +%s` - $starttime))
else
	fail=0
	duration=$maxtime
fi
rm $scratch/$1/dav.log $scratch/$1/dav.err
done
if [ $fail -eq 1 ]
then
	rm $scratch/$1/results.tar.gz
	rmdir $scratch/$1
	rmdir --ignore-fail-on-non-empty $scratch
	echo "timeout when downloading test data"
	exit 1
fi
path=`pwd`
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd /tmp/
tar --exclude='results_test_*_powerprofiling.csv' -xvzf $scratch/$1/results.tar.gz
echo "Downloaded data to $PWD/$1"

# clean up
rm $scratch/$1/results.tar.gz
rmdir $scratch/$1
rmdir --ignore-fail-on-non-empty $scratch
cd $path
else
    echo "Please specify test id"
fi
