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
#include "mb.h"
#include "tim.h"
#include "mb_os_def.h"
#include "top.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// the following handles are defined in the mb_impl_xxx.c files.
extern MB_MSG_TypeDef MB_MSG_Coil;
extern MB_MSG_TypeDef MB_MSG_Disc;
extern MB_MSG_TypeDef MB_MSG_Holding;
extern MB_MSG_TypeDef MB_MSG_Input;

osSemaphoreId semADBusyHandle;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


osThreadId modbusPollingTaskHandle;
osThreadId pidTuningTaskHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartModbusPollingTask(void const * argument);
void StartTaskRegHolding(void const * argument);
void StartPidTuningTask(void const * argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  osSemaphoreDef(semADBusy);
  semADBusyHandle = osSemaphoreCreate(osSemaphore(semADBusy), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(modbusPollingTask, StartModbusPollingTask, osPriorityNormal, 0, 256);
  modbusPollingTaskHandle = osThreadCreate(osThread(modbusPollingTask), NULL);

  osThreadDef(pidTuningTask, StartPidTuningTask, osPriorityAboveNormal, 0, 256);
  pidTuningTaskHandle = osThreadCreate(osThread(pidTuningTask), NULL);

  CreateMbCoilProcTask();
  CreateMbHoldingProcTask();

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  int i = 0;

  // initialize the board.
  Top_Init();
	
  /* Infinite loop */
  for(;;)
  {
    if(i % 20 == 0)
    {
      Top_TurnOnLed();
      osDelay(10);
    }

    if(i % 30 == 0)
    {
      Top_TurnOffLed();
      i = 0;
    }

    i++;

    osDelay(50);

    Top_UpdateStatus();

  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartModbusPollingTask(void const * argument)
{
  eMBInit(MB_RTU, 0X01, 1, 115200, MB_PAR_NONE);
  eMBEnable();
  for(;;)
  {
    eMBPoll();
  }
}

void StartPidTuningTask(void const * argument)
{
  #include "adn8835.h"

  for(;;)
  {
    if(adn8835.IsAutoTuningStarted == 0)
      osDelay(100);
    else
    {
      Top_TecTune();
      osDelay(env->TECConf.SamplingIntervalMs);
    }
    
  }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
