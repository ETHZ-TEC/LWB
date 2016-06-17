export LD_LIBRARY_PATH=/home/$USER/ti/ccsv6/ccs_base/DebugServer/drivers/
#mspdebug tilib "regs" --allow-fw-update
#mspdebug tilib "md 0xff80 0x0080" --allow-fw-update
mspdebug tilib "run" --allow-fw-update