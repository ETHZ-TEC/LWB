export LD_LIBRARY_PATH=/home/$USER/ti/ccsv6/ccs_base/DebugServer/drivers/
#mspdebug tilib "regs" --allow-fw-update
mspdebug tilib "md 0x8000 0x8000"
#mspdebug tilib "run"
