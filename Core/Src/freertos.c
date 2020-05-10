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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum  {
  WAIT_CONFIG_COMMAND,
  WAIT_VALUE,
  WAIT_MENU_NUMBER
} rx_SMType;


typedef enum  {
  SEND_INFO,
  WAIT_INPUT
} globalSMType;

typedef enum  {
  MAX_THR,
  MIN_THR,
  DIM_VALUE
} thr_Type;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_MAX_TEMP         'x'
#define CMD_MIN_TEMP         'n'

#define CMD_DOWN_THR_MIN     's'
#define CMD_UP_THR_MIN       'w'
#define CMD_DOWN_THR_MAX     'a'
#define CMD_UP_THR_MAX       'd'
#define CMD_UP_DIM           'c'
#define CMD_DOWN_DIM         'z'
#define CMD_MENU             'm'


#define THR_STEP 1
#define DIM_STEP 10

#define MAX_DIM_LIMIT 100
#define MIN_DIM_LIMIT 0

#define MAX_THR_LIMIT 50
#define MIN_THR_LIMIT 0

#define VALUE_STR_LENGTH 11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


thr_Type threshold;
rx_SMType rx_state = WAIT_CONFIG_COMMAND;
globalSMType global_state = SEND_INFO;

extern int MaxTempTh, MinTempTh,dimPercentage;
char * led_status[2] =  {"On",  "Off"};

int count = 0;
int value_count = 0;
char temp_value[VALUE_STR_LENGTH];

char * error_msg = "\n\rError,  invalid data was sent. Please send a valid "
                   "temperature (integer numbers)\n\r\n\r";
char * big_int_msg = "\n\rThis number is bigger than the "
                     "expected values\n\r\n\r";
char * thr_error_msg = "\n\rPlease enter a number between %d and %d\n\r\n\r";
char * dim_error_msg = "\n\rPlease enter a number between %d and %d\n\r\n\r";
char * max_thr_msg = "Set Max Temperature Threshold (C): ";
char * min_thr_msg = "Set Min Temperature Threshold (C): ";
char * dim_msg = "Set Dimming Percentage (%%): ";


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
/* Definitions for UART_RX */
osThreadId_t UART_RXHandle;
const osThreadAttr_t UART_RX_attributes = {
  .name = "UART_RX",
  .priority = (osPriority_t) osPriorityHigh,
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
/* Definitions for RXQueue */
osMessageQueueId_t RXQueueHandle;
const osMessageQueueAttr_t RXQueue_attributes = {
  .name = "RXQueue"
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
/* Definitions for menuSemaphore */
osSemaphoreId_t menuSemaphoreHandle;
const osSemaphoreAttr_t menuSemaphore_attributes = {
  .name = "menuSemaphore"
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
void UART_RX_Task(void *argument);

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
  /* creation of menuSemaphore */
  menuSemaphoreHandle = osSemaphoreNew(1, 1, &menuSemaphore_attributes);

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

  /* creation of RXQueue */
  RXQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &RXQueue_attributes);

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

  /* creation of UART_RX */
  UART_RXHandle = osThreadNew(UART_RX_Task, NULL, &UART_RX_attributes);

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
	  osSemaphoreAcquire(menuSemaphoreHandle, osWaitForever);
		if (status == osOK) {

			//send to LCD and UART
			osMessageQueuePut(LCDQueueHandle, &temp, 0U, osWaitForever);

			osMessageQueuePut(UARTQueueHandle, &temp, 0U, osWaitForever);

			osMessageQueuePut(thermostatLEDQueueHandle, &temp, 0U, osWaitForever);


		}
	 osSemaphoreRelease(menuSemaphoreHandle);

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

/* USER CODE BEGIN Header_UART_RX_Task */
/**
* @brief Function implementing the UART_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_RX_Task */
void UART_RX_Task(void *argument)
{
  /* USER CODE BEGIN UART_RX_Task */
  /* Infinite loop */
	uint8_t chr ;
	osStatus_t status;
  for(;;)
  {
	status = osMessageQueueGet(RXQueueHandle, &chr, NULL, osWaitForever);   // wait for message
	if (status == osOK) {

		switch (rx_state) {
			case WAIT_CONFIG_COMMAND:
				print("%c\n\r", chr);
		        switch (chr) {
		            case CMD_MAX_TEMP:
						print(max_thr_msg);
						global_state = WAIT_INPUT;
						rx_state = WAIT_VALUE;
						threshold = MAX_THR;
						osSemaphoreAcquire(menuSemaphoreHandle, osWaitForever);
		                break;
					case CMD_MIN_TEMP:
						print(min_thr_msg);
						global_state = WAIT_INPUT;
						rx_state = WAIT_VALUE;
						threshold = MIN_THR;
						osSemaphoreAcquire(menuSemaphoreHandle, osWaitForever);
						break;
					case CMD_UP_DIM:
						dimPercentage += DIM_STEP;
						if (dimPercentage > MAX_DIM_LIMIT) \
						dimPercentage = MAX_DIM_LIMIT;
						dimmer_update_percentage(dimPercentage);
						break;
					case CMD_DOWN_DIM:
						dimPercentage -= DIM_STEP;
						if (dimPercentage < MIN_DIM_LIMIT) \
						dimPercentage = MIN_DIM_LIMIT;
						dimmer_update_percentage(dimPercentage);
						break;
					case CMD_UP_THR_MAX:
						if (MaxTempTh + 1 <= MAX_THR_LIMIT) MaxTempTh += THR_STEP;
						break;
					case CMD_DOWN_THR_MAX:
						if (MaxTempTh > MinTempTh + 1) MaxTempTh -= THR_STEP;
						break;
					case CMD_UP_THR_MIN:
						if (MinTempTh + 1 < MaxTempTh) MinTempTh += THR_STEP;
						break;
					case CMD_DOWN_THR_MIN:
						if (MinTempTh -1  >= MIN_THR_LIMIT) MinTempTh -= THR_STEP;
						break;
					case CMD_MENU:
						print("\n\rEnter the number of the parameter "
						"you want to configure:\n\r%s%s%s\n\rParameter: ", \
									"1 - Max Temperature Threshold\n\r", \
									"2 - Min Temperature Threshold\n\r", \
									"3 - Dimming Percentage\n\r");
						global_state = WAIT_INPUT;
						rx_state = WAIT_MENU_NUMBER;
						osSemaphoreAcquire(menuSemaphoreHandle, osWaitForever);
						break;
					default:
						print("\n\rCommand not valid. Type 'x' to configure "
						"max \n\rthreshold or 'n' to "
						"configure min threshold\n\r\n\r");
						break;
		            }

		            break;
		        case WAIT_VALUE:
		            if (chr == '\r') {
		                if (value_count > 0) {
		                    temp_value[value_count] = 0;
		                    int val = atoi(temp_value);
		                    value_count = 0;
		                    print("%c\n\r", chr);
		                    switch (threshold) {
		                        case(MAX_THR):
		                            if ((val > MAX_THR_LIMIT) || (val <= MinTempTh)) {
		                                 // return error
		                                print("\n\r");
		                                print(thr_error_msg,  \
		                                  MinTempTh + 1,  MAX_THR_LIMIT);
		                                print(max_thr_msg);
		                            } else {
		                                MaxTempTh = val;
		                                global_state = SEND_INFO;
		                                rx_state = WAIT_CONFIG_COMMAND;
		                                osSemaphoreRelease(menuSemaphoreHandle);
		                                }
		                            break;
		                        case(MIN_THR):
		                            if ((val < MIN_THR_LIMIT) || (val >= MaxTempTh)) {
		                                 // return error
		                                print("\n\r");
		                                print(thr_error_msg,  MIN_THR_LIMIT,  \
		                                  MaxTempTh - 1);
		                                print(min_thr_msg);
		                            } else {
		                                MinTempTh = val;
		                                global_state = SEND_INFO;
		                                rx_state = WAIT_CONFIG_COMMAND;
		                                osSemaphoreRelease(menuSemaphoreHandle);
		                            }
		                            break;
		                        case(DIM_VALUE):
		                            if ((val < MIN_DIM_LIMIT) || \
		                            (val > MAX_DIM_LIMIT)) {
		                                  // return error
		                                print("\n\r");
		                                print(dim_error_msg,  MIN_DIM_LIMIT,  \
		                                  MAX_DIM_LIMIT);
		                                print(dim_msg);
		                            } else {
		                                dimPercentage = val;
		                                dimmer_update_percentage(dimPercentage);
		                                global_state = SEND_INFO;
		                                rx_state = WAIT_CONFIG_COMMAND;
		                                osSemaphoreRelease(menuSemaphoreHandle);
		                            }
		                            break;
		                        default:
		                            break;
		                    }
		                } else {
		                    print("%c\n\r", chr);
		                    print(error_msg);
		                    switch (threshold) {
		                        case MAX_THR:
		                            print(max_thr_msg);
		                            break;
		                        case MIN_THR:
		                            print(min_thr_msg);
		                            break;
		                        case DIM_VALUE:
		                            print(dim_msg);
		                            break;
		                        default:
		                            break;
		                    }
		                }
		            } else if ((chr >= '0') && (chr <= '9')) {
		                if (value_count > VALUE_STR_LENGTH - 2) {
		                    value_count = 0;
		                    print("%c\n\r", chr);
		                    print(big_int_msg);
		                    switch (threshold) {
		                        case MAX_THR:
		                            print(max_thr_msg);
		                            break;
		                        case MIN_THR:
		                            print(min_thr_msg);
		                            break;
		                        case DIM_VALUE:
		                            print(dim_msg);
		                            break;
		                        default:
		                            break;
		                    }

		                } else {  // is a number
		                    print("%c", chr);
		                    temp_value[value_count++] = chr;
		                }
		            } else {
		                value_count = 0;
		                print("%c\n\r", chr);
		                print(error_msg);
		                switch (threshold) {
		                        case MAX_THR:
		                            print(max_thr_msg);
		                            break;
		                        case MIN_THR:
		                            print(min_thr_msg);
		                            break;
		                        case DIM_VALUE:
		                            print(dim_msg);
		                            break;
		                        default:
		                            break;
		                    }
		            }
		            break;
		        case WAIT_MENU_NUMBER:
		            print("%c\r\n",  chr);
		            switch  (chr) {
		                case '1':
		                    print(max_thr_msg);
		                    global_state = WAIT_INPUT;
		                    rx_state = WAIT_VALUE;
		                    threshold = MAX_THR;
		                    break;
		                case '2':
		                    print(min_thr_msg);
		                    global_state = WAIT_INPUT;
		                    rx_state = WAIT_VALUE;
		                    threshold = MIN_THR;
		                    break;
		                case '3':
		                    print(dim_msg);
		                    global_state = WAIT_INPUT;
		                    rx_state = WAIT_VALUE;
		                    threshold = DIM_VALUE;
		                    break;
		                default:
		                    print("Please select a number in the "
		                    "list.\r\n%s%s%s\n\rParameter: ", \
		                                "1 - Max Temperature Threshold\n\r", \
		                                "2 - Min Temperature Threshold\n\r", \
		                                "3 - Dimming Percentage\n\r");
		                    break;
		            }
		            break;
		        default:
		            rx_state = WAIT_CONFIG_COMMAND;
		            break;
		    }

	}

  }
  /* USER CODE END UART_RX_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
