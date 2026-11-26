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

typedef enum
{
  PROT_NONE = 0U,
  PROT_OCP,
  PROT_UVP
} ProtectReason;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_NUM    100U
#define SAMPLE_DROP   10U
/* 电压采样：R3=10K 上 / R6=3.3K 下，分压比 3.3/(10+3.3)=0.2481
 * ADC 满量程 3.3V 对应 VOUT = 3.3/0.2481 = 13.30V，故满量程取 13300mV */
#define VOL_FULL_MV   13300UL
#define CUR_FULL_MA   3300UL
#define ADC_FULL      4095UL
/* 开输出后屏蔽保护判断的时间，等光耦导通 + 电压建立 */
#define PROTECT_BLANK_MS  800U
/* 切档后电压显示静默期，等 PD 重新协商 + VBUS 泄放到目标档 */
#define VOL_SETTLE_MS     1200U

/* 保护阈值档位：可调范围 / 步进 / 1.8V 支路硬件上限 */
#define PROTECT_MAX_MA      1500U
#define PROTECT_MIN_MA      200U
#define PROTECT_STEP_MA     100U
#define PROTECT_1V8_MAX_MA  900U

/* 长按连发：按住多久开始连发、连发间隔 */
#define KEY_REPEAT_DELAY_MS 500U
#define KEY_REPEAT_RATE_MS  150U

/* 主循环节拍 */
#define SAMPLE_PERIOD_MS    50U
#define UI_REFRESH_MS       200U

/* 屏幕坐标：左标签 / 数值列 / 档位列 / 阈值列 */
#define UI_LABEL_X          0U
#define UI_VALUE_X          36U
#define UI_GEAR_X           96U
#define UI_THRESH_X         92U
/* 行（页）：每行 16px 占两页，四行正好铺满 64px */
#define UI_ROW_STATUS       0U
#define UI_ROW_VOL          2U
#define UI_ROW_CUR          4U
#define UI_ROW_PWR          6U
/* 每行的下半页：8px ASCII 数值与 16px 汉字底边对齐 */
#define UI_TXT_STATUS       1U
#define UI_TXT_VOL          3U
#define UI_TXT_CUR          5U
#define UI_TXT_PWR          7U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SystemParam systemparam;

/* 欠压阈值表：下标 = setVolValue - VOL_1V8 */
static const uint16_t underLimitMv[5] = {1000U, 2000U, 3000U, 5000U, 7000U};

static uint16_t adcVolBuf[SAMPLE_NUM];
static uint16_t adcCurBuf[SAMPLE_NUM];
static uint32_t volMv = 0U;
static uint32_t curMa = 0U;
static uint32_t powerMw = 0U;          /* 瞬时功率 mW */
static uint32_t energyUwh = 0U;        /* 累计能量 uWh */
static uint32_t lastEnergyTick = 0U;
static ProtectReason protectReason = PROT_NONE;
static uint32_t tripMa = 0U;           /* 保护动作瞬间的电流 */
static uint32_t tripMv = 0U;           /* 保护动作瞬间的电压 */
static uint32_t lastOpenTick = 0U;     /* 上次开启输出的时刻，用于保护静默期 */
static uint32_t volHoldStart = 0U;     /* 切档时刻，用于电压显示静默期 */
static uint8_t  repeatKey = NoPRESS;   /* 正在连发的按键 */
static uint32_t repeatStartTick = 0U;
static uint32_t lastRepeatTick = 0U;

/* 屏幕内容缓存：与上次发送的字符串比对，未变则不重发 */
static char uiVolStr[8] = {0};
static char uiCurStr[8] = {0};
static char uiThrStr[8] = {0};
static char uiPwrStr[8] = {0};
static char uiEngStr[12] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KeyHandle(uint8_t key);
uint16_t FilterSample(uint16_t *buf, uint8_t len);
void SampleAndCalc(void);
void ProtectCheck(void);
void ProtectTrip(ProtectReason reason);
void DefaultShowUI(void);
void UpdateShowUI(uint8_t refresh);
void ShowProtectDetail(void);
void FormatVol(char *s, uint32_t mv);
void FormatMa(char *s, uint32_t ma);
void FormatPower(char *s);
void FormatEnergy(char *s);
const char *GearName(uint8_t vol);
void StepVol(int8_t dir);
void StepProtect(int8_t dir);
void RepeatHandle(void);
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
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  systemparam.currentPage  = POWER_PAGE;
  systemparam.isOpen       = DISABLE;
  systemparam.setVolValue  = VOL_5V0;
  systemparam.protectValue = 1000U;
  lastEnergyTick = HAL_GetTick();

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
    PowerSwitchPoll();
    keyValue = keyScanf();
    if (keyValue != NoPRESS)
    {
      KeyHandle(keyValue);
    }
    RepeatHandle();

    if ((HAL_GetTick() - lastSampleTick) >= SAMPLE_PERIOD_MS)
    {
      lastSampleTick = HAL_GetTick();
      SampleAndCalc();
      ProtectCheck();
    }

    if ((HAL_GetTick() - lastUiTick) >= UI_REFRESH_MS)
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

/**
  * @brief  内容变化才发送，避免每 200ms 重刷同一行占用 I2C
  */
static void ShowIfChanged(uint8_t x, uint8_t page, const char *s, char *cache)
{
  uint8_t i = 0U;

  while ((s[i] != '\0') && (s[i] == cache[i]))
  {
    i++;
  }

  if ((s[i] == '\0') && (cache[i] == '\0'))
  {
    return;
  }

  OLED_ShowString(x, page, s, 0U);

  i = 0U;
  do
  {
    cache[i] = s[i];
  } while (s[i++] != '\0');
}

/**
  * @brief  清屏后缓存与屏幕不一致，必须同步作废
  */
static void UiCacheReset(void)
{
  uint8_t i;

  for (i = 0U; i < 12U; i++)
  {
    if (i < 8U)
    {
      uiVolStr[i] = '\0';
      uiCurStr[i] = '\0';
      uiThrStr[i] = '\0';
      uiPwrStr[i] = '\0';
    }
    uiEngStr[i] = '\0';
  }
}

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

void FormatPower(char *s)
{
  uint32_t w  = powerMw / 1000U;
  uint32_t wf = (powerMw % 1000U) / 100U;   /* 0.1W 分辨率 */

  s[0] = (w >= 10U) ? (char)('0' + (w / 10U) % 10U) : ' ';
  s[1] = (char)('0' + (w % 10U));
  s[2] = '.';
  s[3] = (char)('0' + wf);
  s[4] = 'W';
  s[5] = '\0';
}

void FormatEnergy(char *s)
{
  uint32_t eWh = energyUwh / 1000000UL;
  uint32_t em  = (energyUwh % 1000000UL) / 10000UL;   /* 0.01Wh 分辨率 */

  s[0] = 'E';
  s[1] = (char)('0' + (eWh / 100U) % 10U);
  s[2] = (char)('0' + (eWh / 10U) % 10U);
  s[3] = (char)('0' + (eWh % 10U));
  s[4] = '.';
  s[5] = (char)('0' + (em / 10U) % 10U);
  s[6] = (char)('0' + (em % 10U));
  s[7] = 'W';
  s[8] = 'H';
  s[9] = '\0';
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
  uint32_t now;
  uint32_t dt;

  for (i = 0U; i < SAMPLE_NUM; i++)
  {
    if (HAL_ADC_Start(&hadc) != HAL_OK)
    {
      HAL_ADC_Stop(&hadc);
      return;   /* 保留上一次有效结果，避免 0 值引发误保护 */
    }
    adcCurBuf[i] = getAdcValue();
    adcVolBuf[i] = getAdcValue();
    HAL_ADC_Stop(&hadc);
  }

  curAvg = FilterSample(adcCurBuf, SAMPLE_NUM);
  volAvg = FilterSample(adcVolBuf, SAMPLE_NUM);

  volMv = ((uint32_t)volAvg * VOL_FULL_MV) / ADC_FULL;
  curMa = ((uint32_t)curAvg * CUR_FULL_MA) / ADC_FULL;
  powerMw = (volMv * curMa) / 1000U;

  now = HAL_GetTick();
  dt = now - lastEnergyTick;
  lastEnergyTick = now;

  /* 只在输出开启且间隔正常时积分，防止阻塞后补算虚高 */
  if ((systemparam.isOpen == ENABLE) && (dt < 1000U))
  {
    energyUwh += (powerMw * dt) / 3600UL;
  }
}

void ProtectCheck(void)
{
  uint8_t  idx;
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
    tripMa = curMa;
    tripMv = volMv;
    ProtectTrip(PROT_OCP);
    return;
  }

  idx = (uint8_t)(systemparam.setVolValue - VOL_1V8);
  underLimit = (idx < 5U) ? underLimitMv[idx] : 0U;

  if (volMv < underLimit)
  {
    tripMa = curMa;
    tripMv = volMv;
    ProtectTrip(PROT_UVP);
  }
}

void ProtectTrip(ProtectReason reason)
{
  ClosePowerSwitch();
  systemparam.isOpen = DISABLE;
  protectReason = reason;
  UpdateShowUI(1U);
}

void DefaultShowUI(void)
{
  OLED_Clear();
  UiCacheReset();
  UpdateShowUI(1U);
}

void UpdateShowUI(uint8_t refresh)
{
  char buf[24];

  if (refresh != 0U)
  {
    uint8_t invPow = (systemparam.currentPage == POWER_PAGE) ? 1U : 0U;
    uint8_t invVol = (systemparam.currentPage == VOL_PAGE) ? 1U : 0U;
    uint8_t invCur = (systemparam.currentPage == CUR_PAGE) ? 1U : 0U;
    const char *gear = GearName(systemparam.setVolValue);
    uint8_t gearLen = 0U;
    uint8_t gearX;

    OLED_ShowCN(UI_LABEL_X, UI_ROW_STATUS, CN_SHU, invPow);
    OLED_ShowCN(16U, UI_ROW_STATUS, CN_CHU, invPow);
    OLED_ShowCN(UI_LABEL_X, UI_ROW_VOL, CN_DIAN, invVol);
    OLED_ShowCN(16U, UI_ROW_VOL, CN_YA, invVol);
    OLED_ShowCN(UI_LABEL_X, UI_ROW_CUR, CN_DIAN, invCur);
    OLED_ShowCN(16U, UI_ROW_CUR, CN_LIU, invCur);
    OLED_ShowCN(UI_LABEL_X, UI_ROW_PWR, CN_GONG, 0U);
    OLED_ShowCN(16U, UI_ROW_PWR, CN_LV, 0U);

    /* 状态行右侧整段清空，保护详情直接画在同一行 */
    OLED_ClearBlock(UI_VALUE_X, UI_ROW_STATUS, 32U);
    OLED_ClearBlock(68U, UI_ROW_STATUS, 32U);
    OLED_ClearBlock(100U, UI_ROW_STATUS, 28U);
    OLED_ShowCN(UI_VALUE_X, UI_ROW_STATUS,
                (systemparam.isOpen == ENABLE) ? CN_KAI : CN_GUAN, 0U);

    while (gear[gearLen] != '\0')
    {
      gearLen++;
    }
    gearX = (uint8_t)(OLED_WIDTH - ((uint16_t)gearLen * OLED_CHAR_W));
    OLED_ClearBlock(UI_GEAR_X, UI_ROW_VOL, 32U);
    OLED_ShowString(gearX, UI_TXT_VOL, gear, 0U);

    ShowProtectDetail();
  }

  if ((volHoldStart != 0U) &&
      ((HAL_GetTick() - volHoldStart) < VOL_SETTLE_MS))
  {
    ShowIfChanged(UI_VALUE_X, UI_TXT_VOL, "--.--V", uiVolStr);
  }
  else
  {
    FormatVol(buf, volMv);
    ShowIfChanged(UI_VALUE_X, UI_TXT_VOL, buf, uiVolStr);
  }

  FormatMa(buf, curMa);
  ShowIfChanged(UI_VALUE_X, UI_TXT_CUR, buf, uiCurStr);

  FormatMa(buf, systemparam.protectValue);
  ShowIfChanged(UI_THRESH_X, UI_TXT_CUR, buf, uiThrStr);

  FormatPower(buf);
  ShowIfChanged(UI_VALUE_X, UI_TXT_PWR, buf, uiPwrStr);

  FormatEnergy(buf);
  ShowIfChanged(72U, UI_TXT_PWR, buf, uiEngStr);
}

void ShowProtectDetail(void)
{
  char buf[11];
  char tmp[8];
  uint8_t i;

  if (protectReason == PROT_NONE)
  {
    return;   /* 状态行右侧已被整体清空 */
  }

  if (protectReason == PROT_OCP)
  {
    FormatMa(tmp, tripMa);
    buf[0] = 'O'; buf[1] = 'C'; buf[2] = 'P'; buf[3] = ' ';
    for (i = 0U; i < 4U; i++)
    {
      buf[4U + i] = tmp[i];
    }
    buf[8] = 'M'; buf[9] = 'A';
  }
  else
  {
    FormatVol(tmp, tripMv);
    buf[0] = 'U'; buf[1] = 'V'; buf[2] = 'P'; buf[3] = ' ';
    for (i = 0U; i < 6U; i++)
    {
      buf[4U + i] = tmp[i];
    }
  }
  buf[10] = '\0';

  OLED_ShowString(56U, UI_TXT_STATUS, buf, 0U);
}

void KeyHandle(uint8_t key)
{
  if (key == SW1PRESS)
  {
    repeatKey = NoPRESS;
    systemparam.currentPage = (systemparam.currentPage >= CUR_PAGE)
                              ? POWER_PAGE
                              : (uint8_t)(systemparam.currentPage + 1U);
    UpdateShowUI(1U);
    return;
  }

  switch (systemparam.currentPage)
  {
    case POWER_PAGE:
      repeatKey = NoPRESS;
      if (key == SW2PRESS)
      {
        if (systemparam.isOpen == ENABLE)
        {
          ClosePowerSwitch();
          systemparam.isOpen = DISABLE;
        }
        else
        {
          protectReason = PROT_NONE;
          OpenPowerSwitch(systemparam.setVolValue);
          systemparam.isOpen = ENABLE;
          lastOpenTick = HAL_GetTick();
          volHoldStart = lastOpenTick;   /* 合闸前电压未知，先显示 --.--V */
        }
        UpdateShowUI(1U);
      }
      break;

    case VOL_PAGE:
      if (key == SW2PRESS)
      {
        StepVol(1);
      }
      else if (key == SW3PRESS)
      {
        StepVol(-1);
      }
      else
      {
        break;
      }
      repeatKey = key;
      repeatStartTick = HAL_GetTick();
      lastRepeatTick = repeatStartTick;
      break;

    case CUR_PAGE:
      if (key == SW2PRESS)
      {
        StepProtect(1);
      }
      else if (key == SW3PRESS)
      {
        StepProtect(-1);
      }
      else
      {
        break;
      }
      repeatKey = key;
      repeatStartTick = HAL_GetTick();
      lastRepeatTick = repeatStartTick;
      break;

    default:
      break;
  }
}

void StepVol(int8_t dir)
{
  if (dir > 0)
  {
    systemparam.setVolValue = (systemparam.setVolValue >= VOL_12V)
                              ? VOL_1V8
                              : (uint8_t)(systemparam.setVolValue + 1U);
  }
  else
  {
    systemparam.setVolValue = (systemparam.setVolValue <= VOL_1V8)
                              ? VOL_12V
                              : (uint8_t)(systemparam.setVolValue - 1U);
  }

  /* 1.8V 支路（TLV75718 限流 1.55A）能力有限，进入该档时压低保护阈值 */
  if ((systemparam.setVolValue == VOL_1V8) &&
      (systemparam.protectValue > PROTECT_1V8_MAX_MA))
  {
    systemparam.protectValue = PROTECT_1V8_MAX_MA;
  }

  volHoldStart = HAL_GetTick();
  if (systemparam.isOpen == ENABLE)
  {
    OpenPowerSwitch(systemparam.setVolValue);
    lastOpenTick = HAL_GetTick();
  }
  UpdateShowUI(1U);
}

void StepProtect(int8_t dir)
{
  uint16_t maxMa = (systemparam.setVolValue == VOL_1V8)
                   ? PROTECT_1V8_MAX_MA
                   : PROTECT_MAX_MA;

  if (dir > 0)
  {
    systemparam.protectValue = (systemparam.protectValue >= maxMa)
                               ? PROTECT_MIN_MA
                               : (uint16_t)(systemparam.protectValue + PROTECT_STEP_MA);
  }
  else
  {
    systemparam.protectValue = (systemparam.protectValue <= PROTECT_MIN_MA)
                               ? maxMa
                               : (uint16_t)(systemparam.protectValue - PROTECT_STEP_MA);
  }

  UpdateShowUI(0U);
}

void RepeatHandle(void)
{
  uint32_t now;

  if (repeatKey == NoPRESS)
  {
    return;
  }

  now = HAL_GetTick();

  if (keyHeld(repeatKey) == 0U)
  {
    repeatKey = NoPRESS;
    return;
  }

  if ((now - repeatStartTick) < KEY_REPEAT_DELAY_MS)
  {
    return;
  }

  if ((now - lastRepeatTick) < KEY_REPEAT_RATE_MS)
  {
    return;
  }

  lastRepeatTick = now;

  if (systemparam.currentPage == VOL_PAGE)
  {
    StepVol((repeatKey == SW2PRESS) ? 1 : -1);
  }
  else if (systemparam.currentPage == CUR_PAGE)
  {
    StepProtect((repeatKey == SW2PRESS) ? 1 : -1);
  }
  else
  {
    repeatKey = NoPRESS;
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
