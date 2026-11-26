/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CFG2_GPIO_Port, CFG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CFG3_GPIO_Port, CFG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CON_VBUS_Pin|CON_3V3_Pin|CON_1V8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CFG1_Pin CFG2_Pin */
  GPIO_InitStruct.Pin = CFG1_Pin|CFG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CFG3_Pin CON_VBUS_Pin CON_3V3_Pin CON_1V8_Pin */
  GPIO_InitStruct.Pin = CFG3_Pin|CON_VBUS_Pin|CON_3V3_Pin|CON_1V8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW3_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/**
  * @brief  CH224K PD 诱骗电平设置（CFG1=PF0, CFG2=PF1, CFG3=PA1）
  * @note   真值表：
  *           5V  -> CFG1=1, CFG2=0, CFG3=0
  *           9V  -> CFG1=0, CFG2=0, CFG3=0
  *           12V -> CFG1=0, CFG2=0, CFG3=1
  * @param  vol: VOL_5V0 / VOL_9V0 / VOL_12V
  */
void setch224(uint8_t vol)
{
  /* 先回 9V 安全档：三脚同时清 0，避免逐位切换时出现未定义/高档中间态 */
  HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CFG2_GPIO_Port, CFG2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CFG3_GPIO_Port, CFG3_Pin, GPIO_PIN_RESET);

  switch (vol)
  {
    case VOL_9V0:
      break;

    case VOL_12V:
      HAL_GPIO_WritePin(CFG3_GPIO_Port, CFG3_Pin, GPIO_PIN_SET);
      break;

    case VOL_5V0:
    default:
      HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, GPIO_PIN_SET);
      break;
  }
}

/**
  * @brief  继电器统一控制（GAQY212GHA，低电平导通）
  * @param  port : CON_xxx_GPIO_Port
  * @param  pin  : CON_xxx_Pin
  * @param  state: OPENPOWER / CLOSEPOWER
  */
void RelaySet(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
  HAL_GPIO_WritePin(port, pin,
                    (state == OPENPOWER) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 异步开输出：记录待合闸的档位，由主循环轮询到点执行 */
static uint8_t  pendingVol    = VOL_5V0;
static uint8_t  pendingActive = 0U;
static uint32_t pendingTick   = 0U;

/**
  * @brief  请求打开指定档位输出（非阻塞，立即返回）
  * @note   先全断，避免旧档仍在导通时 VBUS 跳变直接打到负载上；
  *         设好 PD 档位后只登记，由 PowerSwitchPoll() 在协商完成后合闸。
  */
void OpenPowerSwitch(uint8_t vol)
{
  ClosePowerSwitch();
  setch224(vol);

  pendingVol    = vol;
  pendingTick   = HAL_GetTick();
  pendingActive = 1U;
}

/**
  * @brief  合闸轮询（主循环调用）：PD 协商时间到后只合目标路
  */
void PowerSwitchPoll(void)
{
  if (pendingActive == 0U)
  {
    return;
  }

  if ((HAL_GetTick() - pendingTick) < PD_SETTLE_MS)
  {
    return;
  }

  pendingActive = 0U;

  switch (pendingVol)
  {
    case VOL_1V8: RelaySet(CON_1V8_GPIO_Port, CON_1V8_Pin, OPENPOWER); break;
    case VOL_3V3: RelaySet(CON_3V3_GPIO_Port, CON_3V3_Pin, OPENPOWER); break;
    case VOL_5V0:
    case VOL_9V0:
    case VOL_12V:
    default:      RelaySet(CON_VBUS_GPIO_Port, CON_VBUS_Pin, OPENPOWER); break;
  }
}

/**
  * @brief  关闭全部输出（保护动作 / 待机态），并取消未完成的合闸
  */
void ClosePowerSwitch(void)
{
  pendingActive = 0U;

  RelaySet(CON_1V8_GPIO_Port, CON_1V8_Pin, CLOSEPOWER);
  RelaySet(CON_3V3_GPIO_Port, CON_3V3_Pin, CLOSEPOWER);
  RelaySet(CON_VBUS_GPIO_Port, CON_VBUS_Pin, CLOSEPOWER);
}

/**
  * @brief  按键扫描：20ms 消抖状态机（非阻塞，每调用一次只走一步）
  * @note   原始电平一变就重计时；稳定满 20ms 才更新 stable；
  *         只在稳定后的下降沿返回一次键码，不等松手，不阻塞主循环。
  *         主循环需保证调用周期 << 20ms（例如 10ms 一次）。
  * @retval SW1PRESS / SW2PRESS / SW3PRESS / NoPRESS
  */
uint8_t keyScanf(void)
{
  static const uint16_t      pins[3]   = {SW1_Pin, SW2_Pin, SW3_Pin};
  static GPIO_TypeDef * const ports[3] = {SW1_GPIO_Port, SW2_GPIO_Port, SW3_GPIO_Port};
  static const uint8_t       codes[3] = {SW1PRESS, SW2PRESS, SW3PRESS};
  static uint8_t  raw[3]        = {1U, 1U, 1U};
  static uint8_t  stable[3]     = {1U, 1U, 1U};
  static uint32_t changeTick[3] = {0U, 0U, 0U};
  static uint8_t  inited        = 0U;

  uint32_t now    = HAL_GetTick();
  uint8_t  result = NoPRESS;

  if (inited == 0U)
  {
    for (uint8_t i = 0U; i < 3U; i++)
    {
      changeTick[i] = now;
    }
    inited = 1U;
  }

  for (uint8_t i = 0U; i < 3U; i++)
  {
    uint8_t level = (HAL_GPIO_ReadPin(ports[i], pins[i]) == GPIO_PIN_RESET) ? 0U : 1U;

    if (level != raw[i])
    {
      raw[i] = level;
      changeTick[i] = now;
    }
    else if ((stable[i] != level) && ((now - changeTick[i]) >= KEY_DEBOUNCE_MS))
    {
      stable[i] = level;
      if ((level == 0U) && (result == NoPRESS))
      {
        result = codes[i];
      }
    }
  }

  return result;
}

/**
  * @brief  查询按键当前是否仍被按住（不消抖，仅供长按连发节拍判断）
  * @param  key: SW1PRESS / SW2PRESS / SW3PRESS
  * @retval 1=按住 0=松开或参数非法
  */
uint8_t keyHeld(uint8_t key)
{
  switch (key)
  {
    case SW1PRESS:
      return (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    case SW2PRESS:
      return (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    case SW3PRESS:
      return (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    default:
      return 0U;
  }
}

/* USER CODE END 2 */
