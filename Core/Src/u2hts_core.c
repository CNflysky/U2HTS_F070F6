/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  This file is licensed under GPL V3.
  All rights reserved.
*/

#include "u2hts_core.h"

#include <string.h>

#include "touch-controllers/goodix.h"
#include "usbd_customhid.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

extern u2hts_touch_controller *default_controller;
static u2hts_config *config;

static bool u2hts_irq_status = false;
static u2hts_hid_report u2hts_report = {0x00};
static u2hts_hid_report u2hts_previous_report = {0x00};
static uint16_t u2hts_tp_ids_mask = 0;

static u2hts_led_pattern __unused long_flash_once[] = {
    {.level = GPIO_PIN_RESET, .delay_ms = 1000},
    {.level = GPIO_PIN_SET, .delay_ms = 1000}};

void u2hts_delay_us(uint32_t us) {
  __IO uint32_t currentTicks = SysTick->VAL;
  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = SysTick->LOAD + 1;
  /* Number of ticks to count */
  const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint32_t elapsedTicks = 0;
  __IO uint32_t oldTicks = currentTicks;
  do {
    currentTicks = SysTick->VAL;
    elapsedTicks += (oldTicks < currentTicks)
                        ? tickPerMs + oldTicks - currentTicks
                        : oldTicks - currentTicks;
    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);
}

#define I2C_SDA(x) HAL_GPIO_WritePin(TP_SDA_GPIO_Port, TP_SDA_Pin, x)
#define I2C_SCL(x) HAL_GPIO_WritePin(TP_SCL_NSS_GPIO_Port, TP_SCL_NSS_Pin, x)

#define I2C_SDA_R HAL_GPIO_ReadPin(TP_SDA_GPIO_Port, TP_SDA_Pin)
#define I2C_SCL_R HAL_GPIO_ReadPin(TP_SCL_NSS_GPIO_Port, TP_SCL_NSS_Pin, x)

#define I2C_SDA_I                                                 \
  GPIO_InitTypeDef sda_gpio_in = {.Pin = TP_SDA_Pin,              \
                                  .Mode = GPIO_MODE_INPUT,        \
                                  .Speed = GPIO_SPEED_FREQ_HIGH}; \
  HAL_GPIO_Init(TP_SDA_GPIO_Port, &sda_gpio_in);

#define I2C_SDA_O                                                  \
  GPIO_InitTypeDef sda_gpio_out = {.Pin = TP_SDA_Pin,              \
                                   .Mode = GPIO_MODE_OUTPUT_OD,    \
                                   .Pull = GPIO_NOPULL,            \
                                   .Speed = GPIO_SPEED_FREQ_HIGH}; \
  HAL_GPIO_Init(TP_SDA_GPIO_Port, &sda_gpio_out);

#define I2C_DELAY 5

static void i2c_start() {
  I2C_SDA(1);
  I2C_SCL(1);
  u2hts_delay_us(I2C_DELAY / 2);
  I2C_SDA(0);
  u2hts_delay_us(I2C_DELAY / 2);
  I2C_SCL(0);
}

void i2c_stop() {
  I2C_SDA(0);
  I2C_SCL(1);
  u2hts_delay_us(I2C_DELAY / 2);
  I2C_SCL(1);
  I2C_SDA(1);
}

// true: ack false: nack
static void i2c_ack(bool ack) {
  I2C_SCL(0);
  I2C_SDA(!ack);
  u2hts_delay_us(I2C_DELAY);
  I2C_SCL(1);
  u2hts_delay_us(I2C_DELAY);
  I2C_SCL(0);
  I2C_SDA(1);
}

static bool i2c_wait_ack() {
  uint8_t timeout = 0;
  I2C_SDA(1);
  I2C_SCL(1);
  while (I2C_SDA_R) {
    timeout++;
    if (timeout > 254) {
      i2c_stop();
      return false;
    }
  }
  u2hts_delay_us(I2C_DELAY);
  I2C_SCL(0);
  return true;
}

static void i2c_write_byte(uint8_t byte) {
  I2C_SCL(0);
  for (uint8_t i = 0; i < 8; i++) {
    I2C_SDA((byte & 0x80) ? 1 : 0);
    byte <<= 1;
    u2hts_delay_us(I2C_DELAY);
    I2C_SCL(1);
    u2hts_delay_us(I2C_DELAY);
    I2C_SCL(0);
    u2hts_delay_us(I2C_DELAY);
  }
  u2hts_delay_us(I2C_DELAY);
}

static uint8_t i2c_read_byte(bool ack) {
  uint8_t buf = 0x00;
  I2C_SDA_I;
  for (uint8_t i = 0; i < 8; i++) {
    I2C_SCL(0);
    u2hts_delay_us(I2C_DELAY);
    I2C_SCL(1);
    buf <<= 1;
    if (I2C_SDA_R) buf++;
    u2hts_delay_us(I2C_DELAY);
  }
  I2C_SDA_O;
  i2c_ack(ack);
  return buf;
}

void u2hts_i2c_write(uint8_t slave_addr, uint32_t reg, size_t reg_size,
                     void *data, size_t data_size) {
  uint8_t tx_buf[reg_size + data_size];
  uint32_t reg_be = 0x00;
  switch (reg_size) {
    case sizeof(uint16_t):
      reg_be = U2HTS_SWAP16(reg);
      break;
    case sizeof(uint32_t):
      reg_be = U2HTS_SWAP32(reg);
      break;
    default:
      reg_be = reg;
      break;
  }
  memcpy(tx_buf, &reg_be, reg_size);
  memcpy(tx_buf + reg_size, data, data_size);
  i2c_start();
  i2c_write_byte(slave_addr << 1 | 0);
  i2c_wait_ack();
  for (uint32_t i = 0; i < sizeof(tx_buf); i++) {
    i2c_write_byte(tx_buf[i]);
    i2c_wait_ack();
  }
  i2c_stop();
}

void u2hts_i2c_read(uint8_t slave_addr, uint32_t reg, size_t reg_size,
                    void *data, size_t data_size) {
  uint32_t reg_be = 0x00;
  switch (reg_size) {
    case sizeof(uint16_t):
      reg_be = U2HTS_SWAP16(reg);
      break;
    case sizeof(uint32_t):
      reg_be = U2HTS_SWAP32(reg);
      break;
    default:
      reg_be = reg;
      break;
  }
  uint8_t *reg_be_ptr = (uint8_t *)&reg_be;
  uint8_t *data_ptr = (uint8_t *)data;

  i2c_start();
  i2c_write_byte(slave_addr << 1 | 0);
  i2c_wait_ack();

  for (uint8_t i = 0; i < reg_size; i++) {
    i2c_write_byte(reg_be_ptr[i]);
    i2c_wait_ack();
  }

  i2c_start();
  i2c_write_byte((slave_addr << 1) | 1);
  i2c_wait_ack();
  for (uint32_t i = 0; i < data_size; i++)
    data_ptr[i] = i2c_read_byte((i != data_size - 1));
  i2c_stop();
}

static void u2hts_irq_set(bool enable) {
  enable ? HAL_NVIC_EnableIRQ(TP_INT_EXTI_IRQn)
         : HAL_NVIC_DisableIRQ(TP_INT_EXTI_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  u2hts_irq_set(false);
  u2hts_irq_status = (GPIO_Pin == TP_INT_Pin);
}

inline void u2hts_init(u2hts_config *cfg) {
  config = cfg;
  default_controller->i2c_addr =
      (config->i2c_addr) ? config->i2c_addr : default_controller->i2c_addr;

  // setup controller
  default_controller->operations->setup();
  u2hts_touch_controller_config tc_config =
      default_controller->operations->get_config();

  config->x_max = (config->x_max) ? config->x_max : tc_config.x_max;
  config->y_max = (config->y_max) ? config->y_max : tc_config.y_max;
  config->max_tps = (config->max_tps) ? config->max_tps : tc_config.max_tps;
  u2hts_reset_tpint();
}

static inline void u2hts_handle_touch() {
  u2hts_touch_controller_operations *ops = default_controller->operations;
  memset(&u2hts_report, 0x00, sizeof(u2hts_report));
  for (uint8_t i = 0; i < U2HTS_MAX_TPS; i++) u2hts_report.tp[i].id = 0xFF;
  ops->fetch(config, &u2hts_report);

  uint8_t tp_count = u2hts_report.tp_count;
  u2hts_irq_status = false;
  if (tp_count == 0 && u2hts_previous_report.tp_count == 0) return;

  u2hts_report.scan_time = (uint16_t)HAL_GetTick();

  if (u2hts_previous_report.tp_count != u2hts_report.tp_count) {
    uint16_t new_ids_mask = 0;
    for (uint8_t i = 0; i < tp_count; i++) {
      uint8_t id = u2hts_report.tp[i].id;
      if (id < U2HTS_MAX_TPS) U2HTS_SET_BIT(new_ids_mask, id, 1);
    }

    uint16_t released_ids_mask = u2hts_tp_ids_mask & ~new_ids_mask;
    for (uint8_t i = 0; i < U2HTS_MAX_TPS; i++) {
      if (U2HTS_CHECK_BIT(released_ids_mask, i)) {
        for (uint8_t j = 0; j < U2HTS_MAX_TPS; j++) {
          if (u2hts_previous_report.tp[j].id == i) {
            u2hts_previous_report.tp[j].contact = false;
            u2hts_report.tp[tp_count] = u2hts_previous_report.tp[j];
            tp_count++;
            break;
          }
        }
      }
    }
    u2hts_tp_ids_mask = new_ids_mask;
    u2hts_report.tp_count = tp_count;
  }
  u2hts_report.report_id = 1;

  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&u2hts_report,
                             sizeof(u2hts_report));
  u2hts_previous_report = u2hts_report;
}

inline void u2hts_main() {
  u2hts_irq_set(true);
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, !u2hts_irq_status);
  if (u2hts_irq_status) u2hts_handle_touch();
}
