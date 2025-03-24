#!/bin/bash
openocd -f /usr/share/openocd/scripts/interface/cmsis-dap.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg -c "init" -c "reset halt" -c "flash erase_sector 0 0 31" -c "reset run" -c "exit"
