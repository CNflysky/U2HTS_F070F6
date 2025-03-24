#!/bin/bash
[ -z $1 ] && echo "Usage: $0 <swd/dfu>" && exit 1

case $1 in
dfu)
    dfu-util -d=0483:df11 -a 0 -s 0x08000000:mass-erase:force
    ;;
swd)
    openocd -f /usr/share/openocd/scripts/interface/cmsis-dap.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg -c "init" -c "reset halt" -c "flash erase_sector 0 0 31" -c "reset run" -c "exit"
    ;;
esac
