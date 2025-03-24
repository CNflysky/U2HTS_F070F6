#!/bin/bash
ninja -C build/RelWithDebInfo
openocd -f /usr/share/openocd/scripts/interface/cmsis-dap.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg -c "program build/RelWithDebInfo/U2HTS_F070F6.elf verify reset exit"
