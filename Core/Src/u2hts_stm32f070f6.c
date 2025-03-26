/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  This file is licensed under GPL V3.
  All rights reserved.
*/

#include "u2hts_stm32f070f6.h"

#include "u2hts_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

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

inline bool u2hts_i2c_detect_slave(uint8_t addr) {
  i2c_start();
  i2c_write_byte(addr << 1 | 1);
  bool ret = i2c_wait_ack();
  i2c_stop();
  return ret;
}

inline void u2hts_ts_irq_setup(u2hts_touch_controller *ctrler) {
  HAL_GPIO_DeInit(TP_INT_GPIO_Port, TP_INT_Pin);
  uint32_t irq_flag = 0x00;
  switch (ctrler->irq_flag) {
    case U2HTS_IRQ_TYPE_HIGH:
    case U2HTS_IRQ_TYPE_RISING:
      irq_flag = GPIO_MODE_IT_RISING;
      break;
    case U2HTS_IRQ_TYPE_FALLING:
    case U2HTS_IRQ_TYPE_LOW:
    default:
      irq_flag = GPIO_MODE_IT_FALLING;
      break;
  }
  GPIO_InitTypeDef gpio = {
      .Mode = irq_flag, .Pin = TP_INT_Pin, .Pull = GPIO_PULLUP};
  HAL_GPIO_Init(TP_INT_GPIO_Port, &gpio);
}

inline bool u2hts_usb_report(u2hts_hid_report *report, uint8_t report_id) {
  UNUSED(report_id);
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)report,
                                    sizeof(u2hts_hid_report));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  u2hts_ts_irq_status_set((GPIO_Pin == TP_INT_Pin));
}