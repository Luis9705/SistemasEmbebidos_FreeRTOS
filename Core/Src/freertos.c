/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE BEGIN Variables */

extern int MaxTempTh, MinTempTh,dimPercentage;
char * led_status[2] =  {"On",  "Off"};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for temp_sensing_Ta */
osThreadId_t temp_sensing_TaHandle;
const osThreadAttr_t temp_sensing_Ta_attributes = {
  .name = "temp_sensing_Ta",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for displayTemp */
osThreadId_t displayTempHandle;
const osThreadAttr_t displayTemp_attributes = {
  .name = "displayTemp",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for thermostatLEDs */
osThreadId_t thermostatLEDsHandle;
const osThreadAttr_t thermostatLEDs_attributes = {
  .name = "thermostatLEDs",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for displayLCD */
osThreadId_t displayLCDHandle;
const osThreadAttr_t displayLCD_attributes = {
  .name = "displayLCD",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for displayUART */
osThreadId_t displayUARTHandle;
const osThreadAttr_t displayUART_attributes = {
  .name = "displayUART",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for tempQueue */
osMessageQueueId_t tempQueueHandle;
const osMessageQueueAttr_t tempQueue_attributes = {
  .name = "tempQueue"
};
/* Definitions for LCDQueue */
osMessageQueueId_t LCDQueueHandle;
const osMessageQueueAttr_t LCDQueue_attributes = {
  .name = "LCDQueue"
};
/* Definitions for UARTQueue */
osMessageQueueId_t UARTQueueHandle;
const osMessageQueueAttr_t UARTQueue_attributes = {
  .name = "UARTQueue"
};
/* Definitions for thermostatLEDQueue */
osMessageQueueId_t thermostatLEDQueueHandle;
const osMessageQueueAttr_t thermostatLEDQueue_attributes = {
  .name = "thermostatLEDQueue"
};
/* Definitions for temperatureMutex */
osMutexId_t temperatureMutexHandle;
const osMutexAttr_t temperatureMutex_attributes = {
  .name = "temperatureMutex"
};
/* Definitions for ledMutex */
osMutexId_t ledMutexHandle;
const osMutexAttr_t ledMutex_attributes = {
  .name = "ledMutex"
};
/* Definitions for updateLEDSemaphore */
osSemaphoreId_t updateLEDSemaphoreHandle;
const osSemaphoreAttr_t updateLEDSemaphore_attributes = {
  .name = "updateLEDSemaphore"
};
/* Definitions for timeEventSemaphore */
osSemaphoreId_t timeEventSemaphoreHandle;
const osSemaphoreAttr_t timeEventSemaphore_attributes = {
  .name = "timeEventSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void tempSensingTask(void *argument);
void displayTempTask(void *argument);
void thermostatLEDsTask(void *argument);
void displayLCDTask(void *argument);
void displayUARTTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of temperatureMutex */
  temperatureMutexHandle = osMutexNew(&temperatureMutex_attributes);

  /* creation of ledMutex */
  ledMutexHandle = osMutexNew(&ledMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of updateLEDSemaphore */
  updateLEDSemaphoreHandle = osSemaphoreNew(1, 1, &updateLEDSemaphore_attributes);

  /* creation of timeEventSemaphore */
  timeEventSemaphoreHandle = osSemaphoreNew(1, 1, &timeEventSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of tempQueue */
  tempQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &tempQueue_attributes);

  /* creation of LCDQueue */
  LCDQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &LCDQueue_attributes);

  /* creation of UARTQueue */
  UARTQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &UARTQueue_attributes);

  /* creation of thermostatLEDQueue */
  thermostatLEDQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &thermostatLEDQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of temp_sensing_Ta */
  temp_sensing_TaHandle = osThreadNew(tempSensingTask, NULL, &temp_sensing_Ta_attributes);

  /* creation of displayTemp */
  displayTempHandle = osThreadNew(displayTempTask, NULL, &displayTemp_attributes);

  /* creation of thermostatLEDs */
  thermostatLEDsHandle = osThreadNew(thermostatLEDsTask, NULL, &thermostatLEDs_attributes);

  /* creation of displayLCD */
  displayLCDHandle = osThreadNew(displayLCDTask, NULL, &displayLCD_attributes);

  /* creation of displayUART */
  displayUARTHandle = osThreadNew(displayUARTTask, NULL, &displayUART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

/* USER CODE BEGIN Header_tempSensingTask */
/**
* @brief Function implementing the temp_sensing_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tempSensingTask */
void tempSensingTask(void *argument)
{
  /* USER CODE BEGIN tempSensingTask */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(timeEventSemaphoreHandle, osWaitForever);


	  //sensing temp
	  uint16_t  temp = temp_sensor_read();

	  sensingLED_TOGGLE(); // process data

	  //send to queue
	  osMessageQueuePut(tempQueueHandle, &temp, 0U, osWaitForever);

  }
  /* USER CODE END tempSensingTask */
}

/* USER CODE BEGIN Header_displayTempTask */
/**
* @brief Function implementing the displayTemp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayTempTask */
void displayTempTask(void *argument)
{
  /* USER CODE BEGIN displayTempTask */
  /* Infinite loop */
	uint16_t temp;
	osStatus_t status;

  for(;;)
  {
	  status = osMessageQueueGet(tempQueueHandle, &temp, NULL,  osWaitForever);   // wait for message
		if (status == osOK) {

			//send to LCD and UART
			osMessageQueuePut(LCDQueueHandle, &temp, 0U, osWaitForever);

			osMessageQueuePut(UARTQueueHandle, &temp, 0U, osWaitForever);

			osMessageQueuePut(thermostatLEDQueueHandle, &temp, 0U, osWaitForever);


		}

  }
  /* USER CODE END displayTempTask */
}

/* USER CODE BEGIN Header_thermostatLEDsTask */
/**
* @brief Function implementing the thermostatLEDs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_thermostatLEDsTask */
void thermostatLEDsTask(void *argument)
{
  /* USER CODE BEGIN thermostatLEDsTask */
  /* Infinite loop */
	uint16_t temp ;
	osStatus_t status;
  for(;;)
  {
	status = osMessageQueueGet(thermostatLEDQueueHandle, &temp, NULL, osWaitForever);   // wait for message
	if (status == osOK) {

		(temp > MaxTempTh) ? maxTempLED_ON() : maxTempLED_OFF();

		(temp < MinTempTh) ? minTempLED_ON() : minTempLED_OFF();


	}

  }
  /* USER CODE END thermostatLEDsTask */
}

/* USER CODE BEGIN Header_displayLCDTask */
/**
* @brief Function implementing the displayLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayLCDTask */
void displayLCDTask(void *argument)
{
  /* USER CODE BEGIN displayLCDTask */
  /* Infinite loop */
	uint16_t temp ;
	osStatus_t status;
  for(;;)
  {
	  status = osMessageQueueGet(LCDQueueHandle, &temp, NULL, osWaitForever);   // wait for message
	if (status == osOK) {

		LCD_Set_Cursor(1, 1);
		LCD_printf("Temp: %d%cC", temp, 223);
		LCD_Set_Cursor(2, 1);
		LCD_printf("LED dimm: %d%%", dimPercentage);

	}

  }
  /* USER CODE END displayLCDTask */
}

/* USER CODE BEGIN Header_displayUARTTask */
/**
* @brief Function implementing the displayUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayUARTTask */
void displayUARTTask(void *argument)
{
  /* USER CODE BEGIN displayUARTTask */
  /* Infinite loop */

	uint16_t temp ;
	osStatus_t status;
  for(;;)
  {
	status = osMessageQueueGet(UARTQueueHandle, &temp, NULL, osWaitForever);   // wait for message
	if (status == osOK) {

		print("Temp: %dC,  MaxTempTh: %dC,  MinTempTh: %dC,  "
                "MaxTemp: %s,  MinTemp: %s,  Led Intensity: %d%%\n\r"
				,  temp,  MaxTempTh,  MinTempTh,  \
				led_status[getMaxLED_STATUS()], led_status[getMinLED_STATUS()],  \
				dimPercentage);

	}

  }
  /* USER CODE END displayUARTTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
