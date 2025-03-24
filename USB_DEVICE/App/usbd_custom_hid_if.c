/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v2.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
      0x05, 0x0D,        // (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x09, 0x04,        // (LOCAL)  USAGE              0x000D0004 Touch Screen (Application Collection)  
      0xA1, 0x01,        // (MAIN)   COLLECTION         0x01 Application (Usage=0x000D0004: Page=Digitizer Page, Usage=Touch Screen, Type=Application Collection)
      0x85, 0x01,        //   (GLOBAL) REPORT_ID          0x01 (1)  
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0x35, 0x00,        //   (GLOBAL) PHYSICAL_MINIMUM   0x00 (0)  <-- Info: Consider replacing 35 00 with 34
      0x15, 0x00,        //   (GLOBAL) LOGICAL_MINIMUM    0x00 (0)  <-- Info: Consider replacing 15 00 with 14
      0x55, 0x0E,        //   (GLOBAL) UNIT_EXPONENT      0x0E (Unit Value x 10⁻²)  
      0x65, 0x11,        //   (GLOBAL) UNIT               0x11 Distance in metres [1 cm units] (1=System=SI Linear, 1=Length=Centimetre)  
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap  <-- Error: PHYSICAL_MAXIMUM is undefined
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap  <-- Error: PHYSICAL_MAXIMUM is undefined
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x09, 0x22,        //   (LOCAL)  USAGE              0x000D0022 Finger (Logical Collection)  
      0xA1, 0x02,        //   (MAIN)   COLLECTION         0x02 Logical (Usage=0x000D0022: Page=Digitizer Page, Usage=Finger, Type=Logical Collection)
      0x09, 0x42,        //     (LOCAL)  USAGE              0x000D0042 Tip Switch (Momentary Control)  
      0x25, 0x01,        //     (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)  
      0x75, 0x01,        //     (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x95, 0x07,        //     (GLOBAL) REPORT_COUNT       0x07 (7) Number of fields  
      0x81, 0x03,        //     (MAIN)   INPUT              0x00000003 (7 fields x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x09, 0x51,        //     (LOCAL)  USAGE              0x000D0051 Contact Identifier (Dynamic Value)  
      0x95, 0x01,        //     (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x01,        //     (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page 
      0x26, 0x00, 0x08,  //     (GLOBAL) LOGICAL_MAXIMUM    0x0800 (2048)  
      0x75, 0x10,        //     (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x00010030 X (Dynamic Value)  
      0x46, 0x00, 0x08,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x0800 (2048)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x31,        //     (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x05, 0x0D,        //     (GLOBAL) USAGE_PAGE         0x000D Digitizer Page 
      0x26, 0xFF, 0x00,  //     (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x46, 0xFF, 0x00,  //     (GLOBAL) PHYSICAL_MAXIMUM   0x00FF (255)  
      0x75, 0x08,        //     (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x95, 0x03,        //     (GLOBAL) REPORT_COUNT       0x03 (3) Number of fields  
      0x09, 0x48,        //     (LOCAL)  USAGE              0x000D0048 Width (Dynamic Value)  
      0x09, 0x49,        //     (LOCAL)  USAGE              0x000D0049 Height (Dynamic Value)  
      0x09, 0x30,        //     (LOCAL)  USAGE              0x000D0030 Tip Pressure (Dynamic Value)  
      0x81, 0x02,        //     (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0,              //   (MAIN)   END_COLLECTION     Logical  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00000011,EXP=-2)
      0x27, 0xFF, 0xFF, 0x00, 0x00, //   (GLOBAL) LOGICAL_MAXIMUM    0x0000FFFF (65535)  
      0x75, 0x10,        //   (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field  
      0x55, 0x0C,        //   (GLOBAL) UNIT_EXPONENT      0x0C (Unit Value x 10⁻⁴)  
      0x66, 0x01, 0x10,  //   (GLOBAL) UNIT               0x1001 Time in seconds [1 s units] (1=System=SI Linear, 1=Time=Seconds)  
      0x95, 0x01,        //   (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields  
      0x09, 0x56,        //   (LOCAL)  USAGE              0x000D0056 Scan Time (Dynamic Value)  
      0x81, 0x02,        //   (MAIN)   INPUT              0x00000002 (1 field x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x09, 0x54,        //   (LOCAL)  USAGE              0x000D0054 Contact Count (Dynamic Value)  
      0x25, 0x0A,        //   (GLOBAL) LOGICAL_MAXIMUM    0x0A (10)  
      0x75, 0x08,        //   (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field  
      0x81, 0x02,        //   (MAIN)   INPUT              0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x85, 0x02,        //   (GLOBAL) REPORT_ID          0x02 (2)  
      0x09, 0x55,        //   (LOCAL)  USAGE              0x000D0055 Contact Count Maximum (Static Value)  
      0xB1, 0x02,        //   (MAIN)   FEATURE            0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0x85, 0x03,        //   (GLOBAL) REPORT_ID          0x03 (3)  
      0x06, 0x00, 0xFF,  //   (GLOBAL) USAGE_PAGE         0xFF, 0x00 Vendor-defined 
      0x09, 0xC5,        //   (LOCAL)  USAGE              0xFF, 0x0000C5 <-- Warning: Undocumented usage (document it by inserting 00C5 into file FF00.conf)
      0x26, 0xFF, 0x00,  //   (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)  
      0x97, 0x00, 0x01, 0x00, 0x00, //   (GLOBAL) REPORT_COUNT       0x00000100 (256) Number of fields  <-- Info: Consider replacing 97 00010000 with 96 0001
      0xB1, 0x02,        //   (MAIN)   FEATURE            0x00000002 (256 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
      0xC0               // (MAIN)   END_COLLECTION     Application  <-- Warning: Physical units are still in effect PHYSICAL(MIN=0,MAX=255) UNIT(0x00001001,EXP=-4)
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  UNUSED(event_idx);
  UNUSED(state);
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

