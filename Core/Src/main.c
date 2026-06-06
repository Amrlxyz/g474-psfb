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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "charger_uart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 5
#define ADC2_BUFFER_SIZE 3
#define VPFC_SCALE 241.0   // (3/(3+720))^-1
#define VOUT_SCALE 307.383 // (2.35/(2.35+720))^-1
#define R_FIXED_TEMP 10000
#define A1 0.003354016434680530000f
#define B1 0.000256523550896126f
#define C1 2.60597012072052E-06f
#define D1 6.3292612648746E-08f
#define IOUT_OFFSET 0.98

// Safety Limits
#define LIMIT_VOUT_MAX  600.0
#define LIMIT_VOUT_MIN   20.0
#define LIMIT_IOUT_MAX    7.5
#define LIMIT_IOUT_MIN    0.0
#define LIMIT_POUT_MAX 3000.0
#define LIMIT_VIN_MIN   370.0
#define LIMIT_TEMP_MAX   80.0
#define LIMIT_VBAT_MAX  600.0
#define LIMIT_VBAT_DETECT_MIN   5.0


#define LIMIT_MAX_TOLERANCE 1.05
//#define LIMIT_MIN_TOLERANCE 0.95

volatile float duty = 0.1;
volatile uint32_t deadtime = 0;

volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint16_t adc2_buffer[ADC2_BUFFER_SIZE];

volatile uint8_t psfb_enable = 0;
volatile uint8_t relay_enable = 0;
volatile uint8_t is_startButtonPressed = 0;
volatile uint8_t is_dischargeCommandRx = 0;

float CC_Threshold = 3;
float CC_Threshold_Ramp = 0;
float CV_Threshold = LIMIT_VOUT_MIN;
float CV_Threshold_Ramp = 0;


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
    Measurements temp[3];
} SensorsMeas;

SensorsMeas sensorsMeas = {0};

typedef struct {
    const float *PFCCurrent;
    const float *PFCVoltage;
    const float *OutputVoltage;
    const float *OutputCurrent;
    const float *OutputPower;
    const float *BatteryVoltage;
    const float *temp1;
    const float *temp2;
    const float *temp3;
} SensorsVal;

const volatile SensorsVal sensorsVal = {
    .PFCCurrent = &sensorsMeas.PFCCurrent.val,
    .PFCVoltage = &sensorsMeas.PFCVoltage.val,
    .OutputVoltage = &sensorsMeas.OutputVoltage.val,
    .OutputCurrent = &sensorsMeas.OutputCurrent.val,
    .OutputPower = &sensorsMeas.OutputPower.val,
    .BatteryVoltage = &sensorsMeas.BatteryVoltage.val,
    .temp1 = &sensorsMeas.temp[0].val,
    .temp2 = &sensorsMeas.temp[1].val,
    .temp3 = &sensorsMeas.temp[2].val,
};

typedef enum {
    CHARGER_STATE_IDLE,
    CHARGER_STATE_IDLE_BATT,
    CHARGER_STATE_PRECHARGE,
    CHARGER_STATE_ACTIVE,
    CHARGER_STATE_DISCHARGE,
    CHARGER_STATE_ERROR,
    CHARGER_STATE_TOTAL
} Charger_states;

char *CHARGER_STATE_STRING[CHARGER_STATE_TOTAL] = {
    "IDLE",
    "IDLE_BATT_CONNECTED",
    "PRECHARGE",
    "ACTIVE",
    "DISCHARGE",
    "ERROR",
};

Charger_states volatile charger_state = CHARGER_STATE_IDLE;
Charger_states volatile charger_state_prev = CHARGER_STATE_IDLE;



typedef enum {
    CMD_SET_CV = 'V',
    CMD_SET_CC = 'C',
//    CMD_EN_RELAY = 'R',
//    CMD_EN_PSFB = 'P',
    CMD_START_BUTTON = 'S',
    CMD_FAULT = 'F',
    CMD_DISCHARGE = 'D',
    CMD_INVALID = -1,
} uart_cmds;

typedef enum {
    ERR_UART_TIMEOUT    = 0x0001,
    ERR_LV_FAULT        = 0x0002,
    ERR_VIN_UV          = 0x0004,
    ERR_VIN_OV          = 0x0008,
    ERR_VOUT_OV         = 0x0010,
    ERR_IOUT_OC         = 0x0020,
    ERR_POUT_OP         = 0x0040,
    ERR_TEMP_OT         = 0x0080,
    ERR_VBAT_REVERSE    = 0x0100,
    ERR_VBAT_OV         = 0x0200,
} charger_err_codes;

volatile charger_err_codes charger_error_code = 0;
volatile uint8_t charger_error_lv = 0;
volatile uint8_t charger_error_uartCounter = 0;

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

static void lowpass(Measurements *meas, float x0);
static void fanSpeedUpdate(void);
static void CCCV_check(void);
static void CCCV_ramp(void);
static void enablePinsUpdate(void);


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
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

//  HAL_ADC_Start_IT(&hadc1); // Start ADC Conversion

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buffer, ADC2_BUFFER_SIZE);

  DAC_ChannelConfTypeDef sConfig;
  HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC_CHANNEL_1);
  HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC_CHANNEL_2);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);


  uart_init();

  HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, 1); // This is actually connected to PFC EN

//  uint16_t dac_val = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    printfDma("\\VI:%7.2f, VO:%7.2f, IO:%7.3f, VB:%7.2f, PO:%9.2f, T1:%7.2f, T2:%7.2f, T3:%7.2f, ST:%1d, ER:%04x\n",
                *sensorsVal.PFCVoltage,
                *sensorsVal.OutputVoltage,
                *sensorsVal.OutputCurrent,
                *sensorsVal.BatteryVoltage,
                *sensorsVal.OutputPower,
                *sensorsVal.temp1,
                *sensorsVal.temp2,
                *sensorsVal.temp3,
                charger_state,
                charger_error_code
                );

    CCCV_check();
    fanSpeedUpdate();

    HAL_Delay(100);
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


static uint32_t charger_checkErrors(void){

    uint32_t errorCode = 0;

    if (charger_error_uartCounter > 10){
        errorCode |= ERR_UART_TIMEOUT;
    }

    if (charger_error_lv){
        errorCode |= ERR_LV_FAULT;
    }

    // Check if input voltage suddenly drops
    if (*sensorsVal.PFCVoltage < LIMIT_VIN_MIN){
        errorCode |= ERR_VIN_UV;
    }

    // Check for overvoltage/current/power
    if (*sensorsVal.OutputVoltage > LIMIT_VOUT_MAX * LIMIT_MAX_TOLERANCE){
        errorCode |= ERR_VOUT_OV;
    }
    if (*sensorsVal.OutputCurrent > LIMIT_IOUT_MAX * LIMIT_MAX_TOLERANCE){
        errorCode |= ERR_IOUT_OC;
    }
    if (*sensorsVal.OutputPower > LIMIT_POUT_MAX * LIMIT_MAX_TOLERANCE){
        errorCode |= ERR_POUT_OP;
    }

    // Temp Sensors
    if (*sensorsVal.temp1 > LIMIT_TEMP_MAX ||
        *sensorsVal.temp2 > LIMIT_TEMP_MAX ||
        *sensorsVal.temp3 > LIMIT_TEMP_MAX){
        errorCode |= ERR_TEMP_OT;
    }

    // VBAT CHECK
    if (*sensorsVal.BatteryVoltage < -LIMIT_VBAT_DETECT_MIN){
        errorCode |= ERR_VBAT_REVERSE;
    }
    if (*sensorsVal.BatteryVoltage > LIMIT_VBAT_MAX * LIMIT_MAX_TOLERANCE){
        errorCode |= ERR_VBAT_OV;
    }

    return errorCode;
}


void uart_parseRxFrame(uint8_t* buffer, uint32_t len){

    if (len < 2) // Including '/n'
        return;

    uint8_t cmd = buffer[0];
    float value = atoff((char *)(buffer+2));

    switch (cmd)
    {
    case CMD_SET_CV:
        if (value <= LIMIT_VOUT_MAX && value >= LIMIT_VOUT_MIN){
            CV_Threshold = value;
        }
        break;

    case CMD_SET_CC:
        if (value <= LIMIT_IOUT_MAX && value >= LIMIT_IOUT_MIN){
            CC_Threshold = value;
        }
        break;

//    case CMD_EN_RELAY:
//        if (value > 0){
//            relay_enable = 1;
//        } else {
//            relay_enable = 0;
//        }
//        break;
//
//    case CMD_EN_PSFB:
//        if (value > 0){
//            psfb_enable = 1;
//        } else {
//            psfb_enable = 0;
//        }
//        break;

    case CMD_START_BUTTON:
        is_startButtonPressed = 1;
        break;

    case CMD_DISCHARGE:
        is_dischargeCommandRx = 1;
        break;

    case CMD_FAULT:
        if (value > 0){
            charger_error_lv = value;
        } else {
            charger_error_lv = 0;
        }
        // Reset timeout counter
        charger_error_uartCounter = 0;
        break;

    default:
        charger_error_code = CMD_INVALID;
        break;
    }

    if (cmd){
        if (cmd == CMD_INVALID){
            printfDma("// UART CMD INVALID\n");
        } else {
            printfDma("// UART CMD RECEIVED = [%c][%f]\n", cmd, value);
        }
    }

    memset(buffer, '\0', len);
}


static void enablePinsUpdate(void){
    HAL_GPIO_WritePin(EN_PSFB_GPIO_Port, EN_PSFB_Pin, psfb_enable);
    HAL_GPIO_WritePin(EN_PFC_GPIO_Port,  EN_PFC_Pin,  relay_enable);
}

static void fanSpeedUpdate(void){
    // Variable fan speed control
//    duty = (sensorsMeas.temp[2].val - 30) / 40;
//
//    if (duty > 1)
//        duty = 1;
//    if (duty < 0)
//        duty = 0;
//
    duty = 1.0;

    TIM8->CCR1 = 1700 * duty;
    TIM1->CCR1 = 1700 * duty;
}


static void CCCV_check(void){
    if (CC_Threshold > LIMIT_IOUT_MAX){
        CC_Threshold = LIMIT_IOUT_MAX;
    } else
    if (CC_Threshold < LIMIT_IOUT_MIN){
        CC_Threshold = LIMIT_IOUT_MIN;
    }

    if (CV_Threshold > LIMIT_VOUT_MAX){
        CV_Threshold = LIMIT_VOUT_MAX;
    } else
    if (CV_Threshold < LIMIT_VOUT_MIN){
        CV_Threshold = LIMIT_VOUT_MIN;
    }

    if (CC_Threshold * CV_Threshold > LIMIT_POUT_MAX){
        CC_Threshold = LIMIT_POUT_MAX / CV_Threshold; // Pout limit by adjusting CC
    }
}


static void CCCV_ramp(void){
    // Divide by 1000 for 1kHz
    static const float rampRateCC = 0.5 / 1000.0;
    static const float rampRateCV = 50  / 1000.0;

    if (psfb_enable){
        // CC Ramp
        if (CC_Threshold_Ramp < CC_Threshold){
            CC_Threshold_Ramp += rampRateCC;
            if (CC_Threshold_Ramp > CC_Threshold){
                CC_Threshold_Ramp = CC_Threshold;
            }
        } else if (CC_Threshold_Ramp > CC_Threshold){
            CC_Threshold_Ramp -= rampRateCC;
            if (CC_Threshold_Ramp < CC_Threshold){
                CC_Threshold_Ramp = CC_Threshold;
            }
        }
        // CV Ramp
        if (CV_Threshold_Ramp < CV_Threshold){
            CV_Threshold_Ramp += rampRateCV;
            if (CV_Threshold_Ramp > CV_Threshold){
                CV_Threshold_Ramp = CV_Threshold;
            }
        } else if (CV_Threshold_Ramp > CV_Threshold){
            CV_Threshold_Ramp -= rampRateCV;
            if (CV_Threshold_Ramp < CV_Threshold){
                CV_Threshold_Ramp = CV_Threshold;
            }
        }
    } else {
        // Reset DAC output to 0 if PSFB disabled
        CC_Threshold_Ramp = LIMIT_IOUT_MIN;
        CV_Threshold_Ramp = LIMIT_VOUT_MIN;
    }

    // Clamp result for safety
    if (CC_Threshold_Ramp < LIMIT_IOUT_MIN){
        CC_Threshold_Ramp = LIMIT_IOUT_MIN;
    } else if (CC_Threshold_Ramp > LIMIT_IOUT_MAX){
        CC_Threshold_Ramp = LIMIT_IOUT_MAX;
    }
    if (CV_Threshold_Ramp < LIMIT_VOUT_MIN){
        CV_Threshold_Ramp = LIMIT_VOUT_MIN;
    } else if (CV_Threshold_Ramp > LIMIT_VOUT_MAX){
        CV_Threshold_Ramp = LIMIT_VOUT_MAX;
    }

    uint16_t CC_ThresholdRaw = ((CC_Threshold_Ramp - IOUT_OFFSET) * 0.4 + 0.5) / (3.3/4095);
    DAC1->DHR12R1 = CC_ThresholdRaw;
    // (CC_Threshold * 3.3/4095 - 2.5)  / -0.1;

    uint16_t CV_ThresholdRaw = (CV_Threshold_Ramp * 1.5) / VOUT_SCALE / (3.3/4095);
    DAC1->DHR12R2 = CV_ThresholdRaw;
    // CV_Threshold * 3.3/4095 * 15.12 /  1.5;
}


static void lowpass(Measurements *meas, float x0){

    // https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
    // 1kHz 10Hz cutoff
//    static const float b0 = 0.00094469146f;
//    static const float b1 = 0.00188938292f;
//    static const float b2 = 0.00094469146f;
//    static const float a1 = -1.9111962882f;
//    static const float a2 = 0.9149750541f;

    // 1kHz 2Hz cutoff
    static const float b0 = 0.00003913020209409091;
    static const float b1 = 0.00007826040418818182;
    static const float b2 = 0.00003913020209409091;
    static const float a1 = -1.9822287623675816;
    static const float a2 = 0.982385283175958;

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


volatile uint32_t timerTest = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)     // 1 kHz
    {
        // Handle button press
        if (is_startButtonPressed){
            switch (charger_state){
            case CHARGER_STATE_IDLE:
            case CHARGER_STATE_IDLE_BATT:
                charger_state = CHARGER_STATE_PRECHARGE;
                break;
            default:
                charger_state = CHARGER_STATE_IDLE;
                break;
            }
            is_startButtonPressed = 0;
        }

        // Handle discharge command
        if (is_dischargeCommandRx){
            switch (charger_state){
            case CHARGER_STATE_IDLE:
            case CHARGER_STATE_IDLE_BATT:
                charger_state = CHARGER_STATE_DISCHARGE;
                break;
            default:
                break;
            }
            is_dischargeCommandRx = 0;
        }

        // Update error code
        charger_error_code = charger_checkErrors();
        if (charger_error_code){
            // Bypass error state if in discharge state
            if (charger_state != CHARGER_STATE_DISCHARGE){
                charger_state = CHARGER_STATE_ERROR;
            }
        }

        // Update State
        switch (charger_state){
        case CHARGER_STATE_IDLE:
            relay_enable = 0;
            psfb_enable = 0;
            if (*sensorsVal.BatteryVoltage > LIMIT_VBAT_DETECT_MIN){
                charger_state = CHARGER_STATE_IDLE_BATT;
            }
            break;

        case CHARGER_STATE_IDLE_BATT:
            relay_enable = 0;
            psfb_enable = 0;
            if (*sensorsVal.BatteryVoltage < LIMIT_VBAT_DETECT_MIN){
                charger_state = CHARGER_STATE_IDLE;
            }
            break;

        case CHARGER_STATE_PRECHARGE:
            relay_enable = 0;
            psfb_enable = 1;
            // Precharge check before relay ON
            if (*sensorsVal.OutputVoltage > *sensorsVal.BatteryVoltage * 0.95){
                relay_enable = 1;
                charger_state = CHARGER_STATE_ACTIVE;
            }
            break;

        case CHARGER_STATE_ACTIVE:
            relay_enable = 1;
            psfb_enable = 1;
            if (charger_error_code){
                relay_enable = 0;
                psfb_enable = 0;
                charger_state = CHARGER_STATE_ERROR;

                printfDma("// ERROR DETECTED = [%d]\n", charger_error_code);
            }
            break;

        case CHARGER_STATE_ERROR:
            relay_enable = 0;
            psfb_enable = 0;
            if (!charger_error_code){
                charger_state = CHARGER_STATE_IDLE;
            }
            break;

        case CHARGER_STATE_DISCHARGE:
            relay_enable = 1;
            psfb_enable = 1;
            CC_Threshold = 1;
            CV_Threshold = 60;
            if ((*sensorsVal.PFCVoltage < 20) &&
                (*sensorsVal.OutputVoltage < 20)){
                charger_state = CHARGER_STATE_IDLE;
            }
            break;

        default:
            break;
        }

        CCCV_check();
        CCCV_ramp();
        enablePinsUpdate();

        // Print new state
        if (charger_state_prev != charger_state){
            printfDma("// New State = [%s]\n", CHARGER_STATE_STRING[charger_state]);
            charger_state_prev = charger_state;
        }

        // Increment uart timout counter
        if (charger_error_uartCounter++ == 100)
            charger_error_uartCounter = 100;

        timerTest++;
    }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//    adcVal = HAL_ADC_GetValue(&hadc1); // Read & Update The ADC Result
    if (hadc->Instance == ADC1)
    {
        lowpass(&sensorsMeas.PFCVoltage,      adc_buffer[0] * 3.3/4095 * VPFC_SCALE /  1.5);
        lowpass(&sensorsMeas.OutputVoltage,   adc_buffer[1] * 3.3/4095 * VOUT_SCALE /  1.5);
        lowpass(&sensorsMeas.OutputCurrent,  (adc_buffer[2] * 3.3/4095 - 0.5) / 0.4 + IOUT_OFFSET);
        lowpass(&sensorsMeas.BatteryVoltage,  adc_buffer[3] * 3.3/4095 * VOUT_SCALE /  1.5);
        lowpass(&sensorsMeas.PFCCurrent,      adc_buffer[4]);

        lowpass(&sensorsMeas.OutputPower,     sensorsMeas.OutputVoltage.val * sensorsMeas.OutputCurrent.val);

    }
    
    else if (hadc->Instance == ADC2)
    {
        float Rntc[3];
        float lnR[3];
        float _temp[3];

        for (int i = 0; i < 3; i++){
            if (adc2_buffer[i] != 4095)
                Rntc[i] =  (R_FIXED_TEMP * adc2_buffer[i])/(4095 - adc2_buffer[i]); //lesser variables
            else
                Rntc[i] =  (R_FIXED_TEMP * adc2_buffer[i])/(1);

            lnR[i] = log(Rntc[i]/10000);

            _temp[i] = 1/(A1 + B1 * lnR[i] + C1 * lnR[i] * lnR[i] + D1 * lnR[i] * lnR[i] * lnR[i]) - 273.15;

            if (isnan(_temp[i]))
                break;
            else if (_temp[i] > 200.0f)
                _temp[i] = 200;
            else if (_temp[i] < -10.0f)
                _temp[i] = -10.0f;

            lowpass(&sensorsMeas.temp[i], _temp[i]);
        }

    /**float Vtemp1 = adc2_buffer[0] * 3.3/4095; 
    float Vtemp2 = adc2_buffer[1] * 3.3/4095;
    float Vtemp3 = adc2_buffer[2] * 3.3/4095;

    float Rntc1 =  (R_FIXED_TEMP * Vtemp1)/(3.3 - Vtemp1);
    float Rntc2 =  (R_FIXED_TEMP * Vtemp2)/(3.3 - Vtemp2);
    float Rntc3 =  (R_FIXED_TEMP * Vtemp3)/(3.3 - Vtemp3); **/
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
