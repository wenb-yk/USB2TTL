/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

EventGroupHandle_t *UartRcvEventGroup;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void UserDisplayTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  UartRcvEventGroup = xEventGroupCreate();
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    osThreadDef(userTask01, UserDisplayTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(userTask01), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
extern User_Uart_Parameter user_param_uart[3];
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	EventBits_t xEventGroupValue;
	const EventBits_t xBitsToWaitFor = ( UART1_RECV_BIT | UART2_RECV_BIT | UART3_RECV_BIT );
  /* Infinite loop */
  for(;;)
  {
 #if 1
	if( set == user_param_uart[0].flag.bits.rx_down )
	{
		memcpy(UserTxBufferFS, user_param_uart[0].rx_buff, user_param_uart[0].rx_len);
		CDC_Transmit_FS(UserTxBufferFS, user_param_uart[0].rx_len, CDC_IN_EP);
		user_param_uart[0].flag.bits.rx_down = clear;
	}
	if( set == user_param_uart[1].flag.bits.rx_down )
	{
		memcpy(UserTxBufferFS, user_param_uart[1].rx_buff, user_param_uart[1].rx_len);
		CDC_Transmit_FS(UserTxBufferFS, user_param_uart[1].rx_len, CDC_IN_EP1);
		user_param_uart[1].flag.bits.rx_down = clear;
	}
	if( set == user_param_uart[2].flag.bits.rx_down )
	{
		memcpy(UserTxBufferFS, user_param_uart[2].rx_buff, user_param_uart[2].rx_len);
		CDC_Transmit_FS(UserTxBufferFS, user_param_uart[2].rx_len, CDC_IN_EP2);
		user_param_uart[2].flag.bits.rx_down = clear;
	}
    osDelay(1);
#else
	xEventGroupValue = xEventGroupWaitBits(     UartRcvEventGroup,	/* 事件组的句柄 */
												xBitsToWaitFor,		/* 待测试的事件位 */
												pdTRUE,				/* 满足添加时清除上面的事件位 */
												pdFALSE, 			/* 任意事件位被设置就会退出阻塞态 */
												portMAX_DELAY );	/* 没有超时 */

	if( set == ( xEventGroupValue & UART1_RECV_BIT ) )
	{
		memcpy(UserTxBufferFS, user_param_uart[0].rx_buff, user_param_uart[0].rx_len);
		CDC_Transmit_FS(UserTxBufferFS, user_param_uart[0].rx_len, CDC_IN_EP);
	}
	if( set == ( xEventGroupValue & UART2_RECV_BIT ) )
	{
		memcpy(UserTxBufferFS, user_param_uart[1].rx_buff, user_param_uart[1].rx_len);
		CDC_Transmit_FS(UserTxBufferFS, user_param_uart[1].rx_len, CDC_IN_EP1);
	}
	if( set == ( xEventGroupValue & UART3_RECV_BIT ) )
	{
		memcpy(UserTxBufferFS, user_param_uart[2].rx_buff, user_param_uart[2].rx_len);
		CDC_Transmit_FS(UserTxBufferFS, user_param_uart[2].rx_len, CDC_IN_EP2);
	}

#endif
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void UserDisplayTask(void const * argument)
{
  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_SET);
    osDelay(500);
    HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_RESET);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE END Application */

