#!/bin/bash
ninja -C build/RelWithDebInfo
arm-none-eabi-objcopy -O binary build/RelWithDebInfo/U2HTS_F070F6.elf fw.bin
dfu-util -d=0483:df11 -a 0 -s 0x08000000:leave -D fw.bin
rm fw.bin