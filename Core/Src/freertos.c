/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern uint16_t ADC_ReadChannel(uint32_t channel);
extern uint8_t Moisture_GetPercent(void);
typedef struct {
    float temperature;
    float humidity;
    uint8_t ok;
} DHT_Data;
extern DHT_Data DHT_Read(void);
osMutexId_t lcdMutex;
extern float Ultrasonic_GetDistance_cm(void);

extern void Servo_SetAngle(uint8_t angle);
extern void RGB_Set(uint8_t r, uint8_t g, uint8_t b);
extern void Buzzer_On(void);
extern void Buzzer_Off(void);

extern void LCD_Print(uint8_t col, uint8_t row, const char* s);
extern void LCD_Clear(void);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT_MIN_INTERVAL_MS   2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t g_rawMoist = 0;
uint16_t g_rawLight = 0;
uint8_t  g_moistPct = 0;
DHT_Data g_dht;
float    g_distance_cm = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartSensorTask(void *argument);
void StartControlTask(void *argument);
void StartLCDTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
	/* USER CODE BEGIN Init */
	LCD_Init();
	LCD_Clear();

	/* USER CODE END Init */

	SensorTaskHandle  = osThreadNew(StartSensorTask,  NULL, &SensorTask_attributes);
	ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);
	LCDTaskHandle     = osThreadNew(StartLCDTask,     NULL, &LCDTask_attributes);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;

void StartSensorTask(void *argument)
{
    uint32_t lastDHT = 0;

    for(;;)
    {
        g_rawMoist = ADC_ReadChannel(ADC_CHANNEL_5);
        g_rawLight = ADC_ReadChannel(ADC_CHANNEL_6);
        g_moistPct = Moisture_GetPercent();

        if (osKernelGetTickCount() - lastDHT >= DHT_MIN_INTERVAL_MS) {
            g_dht = DHT_Read();
            lastDHT = osKernelGetTickCount();
        }

        g_distance_cm = Ultrasonic_GetDistance_cm();


        printf("Moist:%d%% Light:%u T:%d H:%d Dist:%d\r\n",
               g_moistPct, g_rawLight,
               (int)g_dht.temperature, (int)g_dht.humidity,
               (int)g_distance_cm);

        osDelay(1000);  // Read sensors every 1s
    }
}

void StartControlTask(void *argument)
{
  for(;;)
  {
    /* Soil moisture -> Servo + RGB */
    if (g_moistPct < 30) {
        Servo_SetAngle(90);
        RGB_Set(0, 0, 255);
    } else if (g_moistPct > 70) {
        Servo_SetAngle(0);
        RGB_Set(0, 255, 0);
    } else {
        Servo_SetAngle(45);
        RGB_Set(255, 255, 0);
    }

    /* Ultrasonic -> Buzzer */
    if (g_distance_cm > 5 ) {
        Buzzer_On();
        osDelay(100);
        Buzzer_Off();
    } else {
        Buzzer_Off();
    }

    osDelay(100);   // 10Hz control loop
  }
}

void StartLCDTask(void *argument)
{
    char line1[17];
    char line2[17];

    // Initialize LCD safely here
    LCD_Init();
    LCD_Clear();

    // Create a mutex for LCD access
    lcdMutex = osMutexNew(NULL);

    for(;;)
    {
        if (g_dht.ok) {
            snprintf(line1, sizeof(line1), "T:%2dC H:%2d%%",
                     (int)g_dht.temperature,
                     (int)g_dht.humidity);
        } else {
            snprintf(line1, sizeof(line1), "T:NA H:NA");
        }

        snprintf(line2, sizeof(line2), "M:%3d%% L:%4u",
                 g_moistPct,
                 g_rawLight);

        // Take mutex before using LCD
        if(osMutexAcquire(lcdMutex, 10) == osOK)
        {
            LCD_Clear();
            LCD_Print(0, 0, line1);
            LCD_Print(0, 1, line2);
            osMutexRelease(lcdMutex);
        }

        osDelay(1000); // Update every 1s
    }
}



/* USER CODE END Application */

