/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// === Globals ===


#define MOISTURE_DRY_RAW   3000.0f
#define MOISTURE_WET_RAW   1500.0f

#define SERVO_MIN_PULSE    1000  // 1 ms
#define SERVO_MAX_PULSE    2000  // 2 ms
#define LCD_ADDR   (0x27 << 1)
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RS        0x01

#define DHT_MIN_INTERVAL_MS 2000

uint16_t ADC_ReadChannel(uint32_t channel);
uint8_t  Moisture_GetPercent(void);
void     Servo_SetAngle(uint8_t angle);
void     RGB_Set(uint8_t r, uint8_t g, uint8_t b);
void     Buzzer_On(void);
void     Buzzer_Off(void);

void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t col, uint8_t row);
void LCD_Print(uint8_t col, uint8_t row, const char *s);

void     DWT_Init(void);
uint32_t micros(void);
void     delay_us(uint32_t us);

// Ultrasonic
float Ultrasonic_GetDistance_cm(void);

// DHT22
typedef struct {
    float temperature;
    float humidity;
    uint8_t ok;
} DHT_Data;

DHT_Data DHT_Read(void);

// UART printf buffer
char uart_buf[128];

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear();

   // Start PWM timers
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Servo
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // RGB R
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // RGB G
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // RGB B
   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Buzzer

   // Enable DWT for microsecond timing
   DWT_Init();

   // Init LCD

   LCD_Print(0, 0, "SMART FLOWERPOT");

   // Initial actuator states
   Servo_SetAngle(0);
   RGB_Set(0, 255, 0);
   Buzzer_Off();

   // DHT timing
   uint32_t last_dht_ms = 0;
   DHT_Data dht = {0};

   char line[17];


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Time now


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// ----- ADC + moisture -----

uint16_t ADC_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

uint8_t Moisture_GetPercent(void)
{
    uint16_t raw = ADC_ReadChannel(ADC_CHANNEL_5);  // PA0

    float percent = 100.0f -
        ((raw - MOISTURE_WET_RAW) / (MOISTURE_DRY_RAW - MOISTURE_WET_RAW) * 100.0f);

    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    return (uint8_t)percent;
}

// ----- Servo / RGB / Buzzer -----

void Servo_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;

    uint32_t pulse = SERVO_MIN_PULSE +
        (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    // scale 0–255 to 0–999 (TIM3 period)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, r * 4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, g * 4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, b * 4);
}

void Buzzer_On(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 250); // 50% duty
}

void Buzzer_Off(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
}

// ----- DWT microsecond timing -----

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((micros() - start) < us) {
        // busy wait
    }
}

// ----- Ultrasonic (HC-SR04) -----

float Ultrasonic_GetDistance_cm(void)
{
    // Send 10us trigger pulse
    HAL_GPIO_WritePin(TRIG_FOR_HCSRO4_GPIO_Port, TRIG_FOR_HCSRO4_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_FOR_HCSRO4_GPIO_Port, TRIG_FOR_HCSRO4_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_FOR_HCSRO4_GPIO_Port, TRIG_FOR_HCSRO4_Pin, GPIO_PIN_RESET);

    // Wait for echo to go high (start), with timeout
    uint32_t start = micros();
    while (HAL_GPIO_ReadPin(ECHO_for_HCSRO4_GPIO_Port, ECHO_for_HCSRO4_Pin) == GPIO_PIN_RESET) {
        if ((micros() - start) > 30000) { // 30 ms timeout
            return -1.0f;
        }
    }

    // Measure high time
    uint32_t echo_start = micros();
    while (HAL_GPIO_ReadPin(ECHO_for_HCSRO4_GPIO_Port, ECHO_for_HCSRO4_Pin) == GPIO_PIN_SET) {
        if ((micros() - echo_start) > 30000) {
            return -1.0f;
        }
    }
    uint32_t echo_end = micros();
    uint32_t pulse_width_us = echo_end - echo_start;

    // Distance (cm) = (time_us / 2) / 29.1
    float distance = (pulse_width_us / 2.0f) / 29.1f;
    return distance;
}

// ----- DHT22 sensor -----

static void DHT_PinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PB10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_PB10_GPIO_Port, &GPIO_InitStruct);
}

static void DHT_PinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PB10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT22_PB10_GPIO_Port, &GPIO_InitStruct);
}

DHT_Data DHT_Read(void)
{
    DHT_Data data = {0};

    uint8_t bits[5] = {0};
    uint32_t t;

    // 1) Start signal for DHT11:
    // Pull low for at least 18 ms, then high for 20–40 µs
    DHT_PinOutput();
    HAL_GPIO_WritePin(DHT22_PB10_GPIO_Port, DHT22_PB10_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);  // 20 ms
    HAL_GPIO_WritePin(DHT22_PB10_GPIO_Port, DHT22_PB10_Pin, GPIO_PIN_SET);
    delay_us(30);
    DHT_PinInput();

    // 2) Wait for DHT response: low (~80us) then high (~80us)
    t = micros();
    while (HAL_GPIO_ReadPin(DHT22_PB10_GPIO_Port, DHT22_PB10_Pin) == GPIO_PIN_SET) {
        if ((micros() - t) > 1000) return data; // no response
    }
    t = micros();
    while (HAL_GPIO_ReadPin(DHT22_PB10_GPIO_Port, DHT22_PB10_Pin) == GPIO_PIN_RESET) {
        if ((micros() - t) > 1000) return data;
    }
    t = micros();
    while (HAL_GPIO_ReadPin(DHT22_PB10_GPIO_Port, DHT22_PB10_Pin) == GPIO_PIN_SET) {
        if ((micros() - t) > 1000) return data;
    }

    // 3) Read 40 bits
    for (int i = 0; i < 40; i++) {
        // wait for line to go high
        t = micros();
        while (HAL_GPIO_ReadPin(DHT22_PB10_GPIO_Port, DHT22_PB10_Pin) == GPIO_PIN_RESET) {
            if ((micros() - t) > 1000) return data;
        }

        // measure length of the high pulse
        uint32_t start = micros();
        while (HAL_GPIO_ReadPin(DHT22_PB10_GPIO_Port, DHT22_PB10_Pin) == GPIO_PIN_SET) {
            if ((micros() - start) > 1000) break;
        }
        uint32_t width = micros() - start;

        int byte = i / 8;
        bits[byte] <<= 1;

        // DHT11: short high (~26-28us) = 0, long high (~70us) = 1
        if (width > 50) {   // threshold ~50us
            bits[byte] |= 1;
        }
    }

    // 4) Check checksum
    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if ((sum & 0xFF) != bits[4]) {
        return data; // checksum error, data.ok stays 0
    }

    // 5) Convert to DHT11 values
    float hum  = bits[0] + bits[1] / 10.0f;
    float temp = bits[2] + bits[3] / 10.0f;

    // Basic sanity check for DHT11 range
    data.ok = 0;
    if (hum >= 0.0f && hum <= 100.0f &&
        temp >= 0.0f && temp <= 50.0f) {
        data.humidity    = hum;
        data.temperature = temp;
        data.ok          = 1;
    }

    return data;
}

// ----- LCD (I2C) -----

static void LCD_SendNibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble << 4) | LCD_BACKLIGHT;
    if (rs) data |= LCD_RS;

    uint8_t buf = data | LCD_ENABLE;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &buf, 1, 10);

    // brief delay
    for (volatile int i = 0; i < 500; i++);

    buf = data & ~LCD_ENABLE;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &buf, 1, 10);
}

static void LCD_SendByte(uint8_t value, uint8_t rs)
{
    LCD_SendNibble(value >> 4, rs);
    LCD_SendNibble(value & 0x0F, rs);
}

static void LCD_Command(uint8_t cmd)
{
    LCD_SendByte(cmd, 0);
}

static void LCD_Data(uint8_t data)
{
    LCD_SendByte(data, 1);
}

void LCD_Init(void)
{
    HAL_Delay(50);

    // Init sequence for 4-bit mode
    LCD_SendNibble(0x03, 0);
    HAL_Delay(5);
    LCD_SendNibble(0x03, 0);
    HAL_Delay(5);
    LCD_SendNibble(0x03, 0);
    HAL_Delay(5);
    LCD_SendNibble(0x02, 0); // 4-bit

    LCD_Command(0x28); // 4-bit, 2 lines, 5x8
    LCD_Command(0x0C); // display on, cursor off
    LCD_Command(0x06); // entry mode
    LCD_Command(0x01); // clear
    HAL_Delay(2);
}

void LCD_Clear(void)
{
    LCD_Command(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    LCD_Command(0x80 | (col + row_offsets[row]));
}

void LCD_Print(uint8_t col, uint8_t row, const char *s)
{
    LCD_SetCursor(col, row);
    while (*s) {
        LCD_Data((uint8_t)*s++);
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
