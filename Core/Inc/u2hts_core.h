/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  This file is licensed under GPL V3.
  All rights reserved.
*/

#ifndef _U2HTS_CORE_H_
#define _U2HTS_CORE_H_

#include <stdbool.h>

#include "main.h"

#define U2HTS_MAP_VALUE(value, src, dest) (((value) * (dest)) / (src))

#define U2HTS_SET_BIT(val, bit, set) \
  ((set) ? ((val) |= (1U << (bit))) : ((val) &= ~(1U << (bit))))

#define U2HTS_CHECK_BIT(val, bit) ((val) & (1 << (bit)))

#define U2HTS_SWAP16(x) __builtin_bswap16(x)

#define U2HTS_SWAP32(x) __builtin_bswap32(x)

#define U2HTS_MAX_TPS 10

#define U2HTS_LOGICAL_MAX 2048

typedef struct __packed {
  bool contact;
  uint8_t id;
  uint16_t x;
  uint16_t y;
  uint8_t width;
  uint8_t height;
  uint8_t pressure;
} u2hts_tp;

typedef struct __packed {
  uint8_t report_id;
  u2hts_tp tp[U2HTS_MAX_TPS];
  uint16_t scan_time;
  uint8_t tp_count;
} u2hts_hid_report;

typedef struct {
  uint16_t x_max;
  uint16_t y_max;
  uint8_t max_tps;
} u2hts_touch_controller_config;

typedef struct {
  uint8_t i2c_addr;
  bool x_y_swap;
  bool x_invert;
  bool y_invert;
  uint16_t x_max;
  uint16_t y_max;
  uint8_t max_tps;
} u2hts_config;

typedef struct {
  void (*setup)();
  u2hts_touch_controller_config (*get_config)();
  void (*fetch)(u2hts_config *cfg, u2hts_hid_report *report);
} u2hts_touch_controller_operations;

typedef struct {
  uint8_t *name;
  uint8_t i2c_addr;
  u2hts_touch_controller_operations *operations;
} u2hts_touch_controller;

void u2hts_delay_us(uint32_t us);
void u2hts_init(u2hts_config *cfg);
void u2hts_main();
void i2c_stop();
void u2hts_i2c_write(uint8_t slave_addr, uint32_t reg, size_t reg_size,
                     void *data, size_t data_size);
void u2hts_i2c_read(uint8_t slave_addr, uint32_t reg, size_t reg_size,
                    void *data, size_t data_size);

inline static void u2hts_tpint_set(bool val) {
  HAL_GPIO_WritePin(TP_INT_GPIO_Port, TP_INT_Pin, val);
}

inline static void u2hts_reset_tpint() {
  HAL_GPIO_DeInit(TP_INT_GPIO_Port, TP_INT_Pin);
  GPIO_InitTypeDef gpio = {
      .Mode = GPIO_MODE_IT_FALLING, .Pin = TP_INT_Pin, .Pull = GPIO_PULLUP};
  HAL_GPIO_Init(TP_INT_GPIO_Port, &gpio);
}

inline static void u2hts_tprst_set(bool val) {
  HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, val);
}

typedef struct __packed {
  GPIO_PinState level;
  uint32_t delay_ms;
} u2hts_led_pattern;

#define U2HTS_LED_DISPLAY_PATTERN(pattern) \
  u2hts_led_display_pattern(pattern, sizeof(pattern))

inline static void u2hts_led_display_pattern(u2hts_led_pattern *pattern,
                                             size_t pattern_len) {
  for (uint8_t i = 0; i < pattern_len / sizeof(u2hts_led_pattern); i++) {
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, pattern[i].level);
    HAL_Delay(pattern[i].delay_ms);
  }
}

#endif