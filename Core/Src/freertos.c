/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"

#include "ms5837_uart.h"
#include "jy901.h"
#include "sona.h"
#include "ins.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01",
  .cb_mem = &myBinarySem01ControlBlock,
  .cb_size = sizeof(myBinarySem01ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t bme280TaskHandle;
const osThreadAttr_t bme280Task_attributes = {
  .name = "bme280Task",
  .stack_size = 128 * 5,
  .priority = (osPriority_t) osPriorityNormal,
};
//osThreadId_t icm42688TaskHandle;
//const osThreadAttr_t icm42688Task_attributes = {
//  .name = "icm42688Task",
//  .stack_size = 128 * 5,
//  .priority = (osPriority_t) osPriorityNormal,
//};
//osThreadId_t ist8310TaskHandle;
//const osThreadAttr_t ist8310Task_attributes = {
//  .name = "ist8310Task",
//  .stack_size = 128 * 5,
//  .priority = (osPriority_t) osPriorityNormal,
//};
osThreadId_t ms5837TaskHandle;
const osThreadAttr_t ms5837Task_attributes = {
  .name = "ms5837Task",
  .stack_size = 128 * 5,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 5,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t jy901TaskHandle;
const osThreadAttr_t jy901Task_attributes = {
  .name = "jy901Task",
  .stack_size = 128 * 5,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t sonaTaskHandle;
const osThreadAttr_t sonaTask_attributes = {
  .name = "sonaTask",
  .stack_size = 128 * 5,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t insTaskHandle;
const osThreadAttr_t insTask_attributes = {
  .name = "insTask",
  .stack_size = 128 * 80, //不要大于100*128
  .priority = (osPriority_t) osPriorityNormal,
};
TaskHandle_t xTaskToNotify = NULL;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Callback01(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	bme280TaskHandle = osThreadNew(bme280_task,NULL,&bme280Task_attributes);
////ist8310TaskHandle = osThreadNew(ist8310_task,NULL,&ist8310Task_attributes);
mainTaskHandle = osThreadNew(main_task,NULL,&mainTask_attributes);
////icm42688TaskHandle = osThreadNew(icm_task,NULL,&icm42688Task_attributes);
ms5837TaskHandle = osThreadNew(ms5837_task,NULL,&ms5837Task_attributes);
jy901TaskHandle = osThreadNew(jy901_task,NULL,&jy901Task_attributes);
//insTaskHandle = osThreadNew(ins_task,NULL,&insTask_attributes);
//sonaTaskHandle = osThreadNew(sona_task,NULL,&sonaTask_attributes);
//xTaskToNotify=xTaskGetHandle("sonaTask");
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

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

