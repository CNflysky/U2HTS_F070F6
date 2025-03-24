# U2HTS_F070F6
Port [U2HTS](https://github.com/CNflysky/U2HTS) to `STM32F070F6P6`.  

# Build
```bash
sudo apt install gcc-arm-none-eabi libnewlib-dev libnewlib-arm-none-eabi make cmake
git clone https://github.com/CNflysky/U2HTS_F070F6.git --depth 1
cd U2HTS_F070F6
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=MinSizeRel
make -j16
```

# Flash
Press `USR button` (close to USB-C port), then plug on the USB-C cable.
```bash
sudo apt install dfu-util
sudo ./flash-dfu.sh
```

# Config
[main.c](./Core/Src/main.c), line 103:
```c
  u2hts_config cfg = {0x00};

  cfg.x_invert = false;
  cfg.y_invert = false;
  cfg.x_y_swap = false;
  
  u2hts_init(&cfg);
```