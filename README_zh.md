# U2HTS_F070F6
[U2HTS](https://github.com/CNflysky/U2HTS)的`STM32F070F6P6`移植版。

# 构建
```bash
sudo apt install gcc-arm-none-eabi libnewlib-dev libnewlib-arm-none-eabi make cmake
git clone https://github.com/CNflysky/U2HTS_F070F6.git --depth 1
cd U2HTS_F070F6
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=MinSizeRel
make -j16
```

# 刷写
按下`用户按键`(靠近Type-C口的那个按键)，再接上Type-C线缆。
```bash
sudo apt install dfu-util
sudo ./flash-dfu.sh
```

# 配置
[main.c](./Core/Src/main.c):
```c
  u2hts_config cfg = {.controller = (uint8_t *)"Goodix",
                      .x_invert = false,
                      .y_invert = false,
                      .x_y_swap = false};
```