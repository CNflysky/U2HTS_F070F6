# U2HTS_F070F6
 [U2HTS](https://github.com/CNflysky/U2HTS) `STM32F070F6P6` Port.  
[zh_CN(简体中文)](./README_zh.md)

# Build
```bash
sudo apt install gcc-arm-none-eabi libnewlib-dev libnewlib-arm-none-eabi make cmake
git clone https://github.com/CNflysky/U2HTS_F070F6.git --depth 1
cd U2HTS_F070F6
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=MinSizeRel
make -j
```

# Flash
Press `USR button` (close to USB-C port), then plug on the USB-C cable.
```bash
sudo apt install dfu-util
sudo ./flash.sh dfu build/U2HTS_F070F6.elf
```

# Config
[main.c](./Core/Src/main.c), line 103:
```c
  u2hts_config cfg = {.controller = (uint8_t *)"Goodix",
                      .x_invert = false,
                      .y_invert = false,
                      .x_y_swap = false};
```