/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t  currentPage;    /* 当前页面 */
  uint8_t  isOpen;         /* ENABLE: 输出已开 / DISABLE: 已关 */
  uint8_t  setVolValue;    /* VOL_1V8 ... VOL_12V */
  uint16_t protectValue;   /* 过流阈值 mA */
} SystemParam;

enum
{
  POWER_PAGE = 0U,
  VOL_PAGE,
  CUR_PAGE
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_NUM    100U
#define SAMPLE_DROP   10U
/* 电压采样：R3=10K 上 / R6=3.3K 下，分压比 3.3/(10+3.3)=0.2481
 * ADC 满量程 3.3V 对应 IN1- = 3.3/0.2481 = 13.30V，故满量程取 13300mV
 * （旧值 16600 会让读数偏高约 25%） */
#define VOL_FULL_MV   13300UL
#define CUR_FULL_MA   3300UL
#define ADC_FULL      4095UL
/* 开输出后屏蔽保护判断的时间，等光耦导通 + 电压建立 */
#define PROTECT_BLANK_MS  800U
/* 切档后电压显示静默期，等 PD 重新协商 + VBUS 泄放到目标档 */
#define VOL_SETTLE_MS     1200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SystemParam systemparam;

static uint16_t adcVolBuf[SAMPLE_NUM];
static uint16_t adcCurBuf[SAMPLE_NUM];
static uint32_t volMv = 0U;
static uint32_t curMa = 0U;
static uint8_t  protectFlag = 0U;
static uint32_t lastOpenTick = 0U;   /* 上次开启输出的时刻，用于保护静默期 */
static uint32_t volHoldStart = 0U;   /* 切档时刻，用于电压显示静默期 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KeyHandle(uint8_t key);
uint16_t FilterSample(uint16_t *buf, uint8_t len);
void SampleAndCalc(void);
void ProtectCheck(void);
void DefaultShowUI(void);
void UpdateShowUI(uint8_t refresh);
void FormatVol(char *s, uint32_t mv);
void FormatCur(char *s, uint32_t ma);
void FormatMa(char *s, uint32_t ma);
const char *GearName(uint8_t vol);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t  keyValue = NoPRESS;
  uint32_t lastSampleTick = 0U;
  uint32_t lastUiTick = 0U;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc);

  systemparam.currentPage  = POWER_PAGE;
  systemparam.isOpen       = DISABLE;
  systemparam.setVolValue  = VOL_5V0;
  systemparam.protectValue = 1000U;

  setch224(VOL_5V0);
  ClosePowerSwitch();

  OLED_Init();
  DefaultShowUI();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    keyValue = keyScanf();
    if (keyValue != NoPRESS)
    {
      KeyHandle(keyValue);
    }

    if ((HAL_GetTick() - lastSampleTick) >= 50U)
    {
      lastSampleTick = HAL_GetTick();
      SampleAndCalc();
      ProtectCheck();
    }

    if ((HAL_GetTick() - lastUiTick) >= 200U)
    {
      lastUiTick = HAL_GetTick();
      UpdateShowUI(0U);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void FormatVol(char *s, uint32_t mv)
{
  uint32_t v = mv / 10U;

  /* 十位为 0 时留空格，避免 5V 显示成 "05.00V" */
  s[0] = (v >= 1000U) ? (char)('0' + (v / 1000U) % 10U) : ' ';
  s[1] = (char)('0' + (v / 100U) % 10U);
  s[2] = '.';
  s[3] = (char)('0' + (v / 10U) % 10U);
  s[4] = (char)('0' + (v % 10U));
  s[5] = 'V';
  s[6] = '\0';
}

void FormatCur(char *s, uint32_t ma)
{
  s[0] = (ma >= 1000U) ? (char)('0' + (ma / 1000U) % 10U) : ' ';
  s[1] = (ma >= 100U)  ? (char)('0' + (ma / 100U) % 10U)  : ' ';
  s[2] = (ma >= 10U)   ? (char)('0' + (ma / 10U) % 10U)   : ' ';
  s[3] = (char)('0' + (ma % 10U));
  s[4] = 'M';
  s[5] = 'A';
  s[6] = '\0';
}

void FormatMa(char *s, uint32_t ma)
{
  s[0] = (ma >= 1000U) ? (char)('0' + (ma / 1000U) % 10U) : ' ';
  s[1] = (char)('0' + (ma / 100U) % 10U);
  s[2] = (char)('0' + (ma / 10U) % 10U);
  s[3] = (char)('0' + (ma % 10U));
  s[4] = 'M';
  s[5] = 'A';
  s[6] = '\0';
}

const char *GearName(uint8_t vol)
{
  switch (vol)
  {
    case VOL_1V8: return "1.8V";
    case VOL_3V3: return "3.3V";
    case VOL_5V0: return "5.0V";
    case VOL_9V0: return "9.0V";
    case VOL_12V: return "12.0V";
    default:      return "----";
  }
}

uint16_t FilterSample(uint16_t *buf, uint8_t len)
{
  uint8_t  i, j;
  uint16_t tmp;
  uint32_t sum = 0U;

  for (i = 0U; i < (uint8_t)(len - 1U); i++)
  {
    for (j = 0U; j < (uint8_t)(len - 1U - i); j++)
    {
      if (buf[j] > buf[j + 1U])
      {
        tmp = buf[j];
        buf[j] = buf[j + 1U];
        buf[j + 1U] = tmp;
      }
    }
  }

  for (i = SAMPLE_DROP; i < (uint8_t)(len - SAMPLE_DROP); i++)
  {
    sum += buf[i];
  }

  return (uint16_t)(sum / (uint32_t)(len - (2U * SAMPLE_DROP)));
}

void SampleAndCalc(void)
{
  uint16_t i;
  uint16_t volAvg;
  uint16_t curAvg;

  for (i = 0U; i < SAMPLE_NUM; i++)
  {
    HAL_ADC_Start(&hadc);
    adcCurBuf[i] = getAdcValue();
    adcVolBuf[i] = getAdcValue();
    HAL_ADC_Stop(&hadc);
  }

  curAvg = FilterSample(adcCurBuf, SAMPLE_NUM);
  volAvg = FilterSample(adcVolBuf, SAMPLE_NUM);

  volMv = ((uint32_t)volAvg * VOL_FULL_MV) / ADC_FULL;
  curMa = ((uint32_t)curAvg * CUR_FULL_MA) / ADC_FULL;
}

void ProtectCheck(void)
{
  uint32_t underLimit;

  if (systemparam.isOpen != ENABLE)
  {
    return;
  }

  /* 刚开输出时电压尚未建立，跳过判断，避免误触发欠压保护 */
  if ((HAL_GetTick() - lastOpenTick) < PROTECT_BLANK_MS)
  {
    return;
  }

  if (curMa > (uint32_t)systemparam.protectValue)
  {
    ClosePowerSwitch();
    systemparam.isOpen = DISABLE;
    protectFlag = 1U;
    UpdateShowUI(1U);
    return;
  }

  switch (systemparam.setVolValue)
  {
    case VOL_1V8: underLimit = 1000U; break;
    case VOL_3V3: underLimit = 2000U; break;
    case VOL_5V0: underLimit = 3000U; break;
    case VOL_9V0: underLimit = 5000U; break;
    case VOL_12V: underLimit = 7000U; break;
    default:      underLimit = 0U;    break;
  }

  if (volMv < underLimit)
  {
    ClosePowerSwitch();
    systemparam.isOpen = DISABLE;
    protectFlag = 1U;
    UpdateShowUI(1U);
  }
}

void DefaultShowUI(void)
{
  OLED_Clear();
  UpdateShowUI(1U);
}

void UpdateShowUI(uint8_t refresh)
{
  char buf[8];

  if (refresh != 0U)
  {
    uint8_t invPow = (systemparam.currentPage == POWER_PAGE) ? 1U : 0U;
    uint8_t invVol = (systemparam.currentPage == VOL_PAGE) ? 1U : 0U;
    uint8_t invCur = (systemparam.currentPage == CUR_PAGE) ? 1U : 0U;
    const char *gear = GearName(systemparam.setVolValue);
    uint8_t gearLen = 0U;
    uint8_t gearX;

    OLED_ShowCN(0U,  0U, CN_SHU,  invPow);
    OLED_ShowCN(16U, 0U, CN_CHU,  invPow);
    OLED_ShowCN(0U,  3U, CN_DIAN, invVol);
    OLED_ShowCN(16U, 3U, CN_YA,   invVol);
    OLED_ShowCN(0U,  6U, CN_DIAN, invCur);
    OLED_ShowCN(16U, 6U, CN_LIU,  invCur);

    OLED_ClearBlock(36U, 0U, 16U);
    OLED_ShowCN(36U, 0U, (systemparam.isOpen == ENABLE) ? CN_KAI : CN_GUAN, 0U);

    OLED_ClearBlock(96U, 0U, 32U);
    if (protectFlag != 0U)
    {
      OLED_ShowCN(96U,  0U, CN_BAO, 0U);
      OLED_ShowCN(112U, 0U, CN_HU,  0U);
    }

    while (gear[gearLen] != '\0')
    {
      gearLen++;
    }
    gearX = (uint8_t)(OLED_WIDTH - ((uint16_t)gearLen * OLED_CHAR_W));
    OLED_ClearBlock(96U, 4U, 32U);
    OLED_ShowString(gearX, 4U, gear, 0U);
  }

  if ((volHoldStart != 0U) &&
      ((HAL_GetTick() - volHoldStart) < VOL_SETTLE_MS))
  {
    OLED_ShowString(36U, 4U, "--.--V", 0U);
  }
  else
  {
    FormatVol(buf, volMv);
    OLED_ShowString(36U, 4U, buf, 0U);
  }

  FormatCur(buf, curMa);
  OLED_ShowString(36U, 7U, buf, 0U);

  FormatMa(buf, systemparam.protectValue);
  OLED_ShowString(92U, 7U, buf, 0U);
}

void KeyHandle(uint8_t key)
{
  if (key == SW1PRESS)
  {
    systemparam.currentPage = (systemparam.currentPage >= CUR_PAGE)
                              ? POWER_PAGE
                              : (uint8_t)(systemparam.currentPage + 1U);
    UpdateShowUI(1U);
    return;
  }

  switch (systemparam.currentPage)
  {
    case POWER_PAGE:
      if (key == SW2PRESS)
      {
        if (systemparam.isOpen == ENABLE)
        {
          ClosePowerSwitch();
          systemparam.isOpen = DISABLE;
        }
        else
        {
          protectFlag = 0U;
          OpenPowerSwitch(systemparam.setVolValue);
          systemparam.isOpen = ENABLE;
          lastOpenTick = HAL_GetTick();
        }
        UpdateShowUI(1U);
      }
      break;

    case VOL_PAGE:
      if (key == SW2PRESS)
      {
        systemparam.setVolValue = (systemparam.setVolValue >= VOL_12V)
                                  ? VOL_1V8
                                  : (uint8_t)(systemparam.setVolValue + 1U);
      }
      else if (key == SW3PRESS)
      {
        systemparam.setVolValue = (systemparam.setVolValue <= VOL_1V8)
                                  ? VOL_12V
                                  : (uint8_t)(systemparam.setVolValue - 1U);
      }
      else
      {
        break;
      }
      volHoldStart = HAL_GetTick();
      if (systemparam.isOpen == ENABLE)
      {
        OpenPowerSwitch(systemparam.setVolValue);
        lastOpenTick = HAL_GetTick();
      }
      UpdateShowUI(1U);
      break;

    case CUR_PAGE:
      if (key == SW2PRESS)
      {
        systemparam.protectValue = (systemparam.protectValue >= 1500U)
                                   ? 200U
                                   : (systemparam.protectValue + 100U);
      }
      else if (key == SW3PRESS)
      {
        systemparam.protectValue = (systemparam.protectValue <= 200U)
                                   ? 1500U
                                   : (systemparam.protectValue - 100U);
      }
      else
      {
        break;
      }
      UpdateShowUI(0U);
      break;

    default:
      break;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
