/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* GAQY212GHA 固态继电器：IO 输出低电平 -> 光耦导通 -> 该路电源接到输出端 */
#define OPENPOWER               0x20U
#define CLOSEPOWER              0x21U

/* 输出档位编码 */
#define VOL_1V8                 0x10U
#define VOL_3V3                 0x11U
#define VOL_5V0                 0x12U
#define VOL_9V0                 0x13U
#define VOL_12V                 0x14U

/* 按键返回值 */
#define NoPRESS                 0xFFU
#define SW1PRESS                0x04U
#define SW2PRESS                0x05U
#define SW3PRESS                0x06U

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

void setch224(uint8_t vol);
void control1V8(uint8_t state);
void control3V3(uint8_t state);
void controlVBUS(uint8_t state);
void OpenPowerSwitch(uint8_t vol);
void ClosePowerSwitch(void);
uint8_t keyScanf(void);
void keyTick(void);
uint8_t keyGetEvent(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

