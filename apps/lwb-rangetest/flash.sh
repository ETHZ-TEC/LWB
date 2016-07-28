export LD_LIBRARY_PATH=/home/$USER/ti/ccsv6/ccs_base/DebugServer/drivers/
if [ -z $1 ]
  then
    mspdebug tilib "prog lwb.hex"
  else
    mspdebug tilib "prog lwb.hex" --allow-fw-update -d $1
fi