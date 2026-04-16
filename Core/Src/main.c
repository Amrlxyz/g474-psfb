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
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 5
#define VPFC_SCALE 241.0   // (3/(3+720))^-1
#define VOUT_SCALE 307.383 // (2.35/(2.35+720))^-1
#define IOUT_OFFSET 0.98

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile float duty = 0.1;
volatile uint32_t deadtime = 17;
volatile uint8_t enable_update = 1;

volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

volatile uint8_t psfb_enable = 1;
volatile uint8_t relay_enable = 0;

float CC_Threshold = 2;
float CV_Threshold = 40;

float pfc_voltage_nominal = 40;
float pfc_voltage_tolerance = 0.1;


typedef struct {
    union {
        float val;
        float y0;   // Current value
    };
    float y1;
    float y2;
    float x0;   // Current measurement
    float x1;
    float x2;
} Measurements;


typedef struct {
    Measurements PFCCurrent;
    Measurements PFCVoltage;
    Measurements OutputVoltage;
    Measurements OutputCurrent;
    Measurements OutputPower;
    Measurements BatteryVoltage;
} Sensors;

Sensors sensors = {0};


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim6);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

//  HAL_ADC_Start_IT(&hadc1); // Start ADC Conversion

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);


  DAC_ChannelConfTypeDef sConfig;
  HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC_CHANNEL_1);
  HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC_CHANNEL_2);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);

//  uint16_t dac_val = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (enable_update){
        uint8_t buff[128];
        uint16_t buffSize = sprintf((char *)buff, "Vin:%7.2f, Vout:%7.2f, Iout:%7.3f, Vbat:%7.2f, Pout:%7.2f\n",
                                    sensors.PFCVoltage.val,
                                    sensors.OutputVoltage.val,
                                    sensors.OutputCurrent.val,
                                    sensors.BatteryVoltage.val,
                                    sensors.OutputPower.val);

        HAL_UART_Transmit(&huart2, buff, buffSize, HAL_MAX_DELAY);

//        enable_update = 0;
        HAL_Delay(100);
    }

    uint16_t CC_ThresholdRaw = ((CC_Threshold - IOUT_OFFSET) * 0.4 + 0.5) / (3.3/4095);
    DAC1->DHR12R1 = CC_ThresholdRaw;
    // (CC_Threshold * 3.3/4095 - 2.5)  / -0.1;

    uint16_t CV_ThresholdRaw = (CV_Threshold * 1.5) / VOUT_SCALE / (3.3/4095);
    DAC1->DHR12R2 = CV_ThresholdRaw;
    // CV_Threshold * 3.3/4095 * 15.12 /  1.5;


    if (sensors.PFCVoltage.val < pfc_voltage_nominal * (1+pfc_voltage_tolerance) &&
        sensors.PFCVoltage.val > pfc_voltage_nominal * (1-pfc_voltage_tolerance) &&
        psfb_enable){

        HAL_GPIO_WritePin(EN_PSFB_GPIO_Port, EN_PSFB_Pin, 1);
    } else {
        HAL_GPIO_WritePin(EN_PSFB_GPIO_Port, EN_PSFB_Pin, 0);
    }


    HAL_GPIO_WritePin(EN_PFC_GPIO_Port,  EN_PFC_Pin,  relay_enable);

//    TIM8->CCR1 = (duty*1700)/2;
//    HAL_TIMEx_ConfigDeadTime(&htim8, deadtime);
//
//    TIM1->CCR1 = (duty*1700)/2;
//    HAL_TIMEx_ConfigDeadTime(&htim1, deadtime);

//        enable_update = 0;

//    if (dac_val > 4095) dac_val = 0;
//    DAC1->DHR12R2 = dac_val++;

    HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

volatile uint32_t timerTest = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        timerTest++;
    }
}

void lowpass(Measurements *meas, float x0){

    // https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
    // 1kHz 10Hz cutoff
    static const float b0 = 0.00094469146f;
    static const float b1 = 0.00188938292f;
    static const float b2 = 0.00094469146f;
    static const float a1 = -1.9111962882f;
    static const float a2 = 0.9149750541f;

    // Shift Values
    meas->x2 = meas->x1;
    meas->x1 = meas->x0;
    meas->y2 = meas->y1;
    meas->y1 = meas->y0;

    meas->x0 = x0;          // Save current measurement

    meas->y0 = meas->x0 * b0 +
               meas->x1 * b1 +
               meas->x2 * b2 -
               meas->y1 * a1 -
               meas->y2 * a2;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//    adcVal = HAL_ADC_GetValue(&hadc1); // Read & Update The ADC Result

    lowpass(&sensors.PFCVoltage,      adc_buffer[0] * 3.3/4095 * VPFC_SCALE /  1.5);
    lowpass(&sensors.OutputVoltage,   adc_buffer[1] * 3.3/4095 * VOUT_SCALE /  1.5);
    lowpass(&sensors.OutputCurrent,  (adc_buffer[2] * 3.3/4095 - 0.5) / 0.4 + IOUT_OFFSET);
    lowpass(&sensors.BatteryVoltage,  adc_buffer[3] * 3.3/4095 * VOUT_SCALE /  1.5);
    lowpass(&sensors.PFCCurrent,      adc_buffer[4]);

    lowpass(&sensors.OutputPower,     sensors.OutputVoltage.val * sensors.OutputCurrent.val);

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
