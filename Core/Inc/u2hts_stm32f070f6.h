/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  This file is licensed under GPL V3.
  All rights reserved.
*/

#ifndef _U2HTS_STM32F070F6_H_
#define _U2HTS_STM32F070F6_H_

#include "main.h"
#include "stdbool.h"
#include "usbd_customhid.h"

#define U2HTS_ENABLE_LED
#define U2HTS_ENABLE_PERSISTENT_CONFIG
#define U2HTS_ENABLE_BUTTON

#define U2HTS_CONFIG_TIMEOUT 3 * 1000  // 3 s

#define U2HTS_LOG_LEVEL -1  // No stdio attached

inline static void u2hts_delay_ms(uint32_t ms) { HAL_Delay(ms); }
inline static void u2hts_delay_us(uint32_t us) {
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

#define U2HTS_SWAP16(x) __builtin_bswap16(x)

#define U2HTS_SWAP32(x) __builtin_bswap32(x)

inline static void u2hts_tpint_set(bool value) {
  HAL_GPIO_WritePin(TP_INT_GPIO_Port, TP_INT_Pin, value);
}

inline static void u2hts_tprst_set(bool value) {
  HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, value);
}

inline static bool u2hts_usb_init() {
  MX_USB_DEVICE_Init();
  return true;
}
inline static uint16_t u2hts_get_scan_time() { return (uint16_t)HAL_GetTick(); }

inline static void u2hts_led_set(bool on) {
  // low level is on
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, !on);
}

#define U2HTS_CONFIG_STORAGE_OFFSET 0x08007C00UL  // last page

inline static void u2hts_write_config(uint16_t cfg) {
  HAL_FLASH_Unlock();
  uint32_t e = 0;
  FLASH_EraseInitTypeDef erase = {.NbPages = 1,
                                  .PageAddress = U2HTS_CONFIG_STORAGE_OFFSET,
                                  .TypeErase = FLASH_TYPEERASE_PAGES};
  HAL_FLASHEx_Erase(&erase, &e);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, U2HTS_CONFIG_STORAGE_OFFSET,
                    cfg);
  HAL_FLASH_Lock();
}

inline static uint16_t u2hts_read_config() {
  return *(uint16_t *)U2HTS_CONFIG_STORAGE_OFFSET;
}

inline static void u2hts_ts_irq_set(bool enable) {
  enable ? HAL_NVIC_EnableIRQ(TP_INT_EXTI_IRQn)
         : HAL_NVIC_DisableIRQ(TP_INT_EXTI_IRQn);
}

inline static bool u2hts_read_button() {
  // default low
  return HAL_GPIO_ReadPin(USR_KEY_GPIO_Port, USR_KEY_Pin);
}

#endif