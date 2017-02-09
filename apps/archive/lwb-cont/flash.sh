export LD_LIBRARY_PATH=/home/$USER/ti/ccsv6/ccs_base/DebugServer/drivers/
if [ -z $1 ]
  then
    mspdebug tilib "prog testapp.hex" --allow-fw-update
  else
    mspdebug tilib "prog testapp.hex" --allow-fw-update -d $1
fi