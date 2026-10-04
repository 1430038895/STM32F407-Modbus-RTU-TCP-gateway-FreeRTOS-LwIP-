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
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "iwdg.h"
#include "mb_rtu.h"
#include "mb_tcp.h"
#include "mb_gateway.h"
#include "joystick.h"
#include "ui.h"
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
/* ---- 任务句柄与属性（name / stack_size[字节] / priority） ---- */

/** Modbus TCP 服务器任务（上行，对电脑） */
osThreadId_t tcpTaskHandle;
const osThreadAttr_t tcpTask_attributes = {
  .name = "tcpTask",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityBelowNormal, /* 低于 defaultTask，保证 LWIP 先初始化 */
};

/** Modbus RTU 主站轮询任务（下行，对从机） */
osThreadId_t rs485TaskHandle;
const osThreadAttr_t rs485Task_attributes = {
  .name = "rs485Task",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityAboveNormal, /* 高于屏幕任务：轮询时不被屏幕打断 */
};

/** 看门狗任务（检查轮询任务心跳，健康才喂狗） */
osThreadId_t wdTaskHandle;
const osThreadAttr_t wdTask_attributes = {
  .name = "wdTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityAboveNormal, /* 高于轮询/网络任务，保证按时喂狗 */
};

/** 数据导出任务（TCP 5000，给 Python / 屏幕读全部点） */
osThreadId_t dumpTaskHandle;
const osThreadAttr_t dumpTask_attributes = {
  .name = "dumpTask",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal, /* 要能抢过轮询任务，及时应答 */
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTcpTask(void *argument);
void StartRs485Task(void *argument);
void StartWatchdogTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
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
  /* 初始化各模块（内部各自创建所需的互斥锁） */
  mbrtu_init();     /* RS485 总线互斥锁 */
  mbgw_init();      /* 缓存互斥锁 */
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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 创建自定义任务 */
  tcpTaskHandle = osThreadNew(StartTcpTask, NULL, &tcpTask_attributes);
  rs485TaskHandle = osThreadNew(StartRs485Task, NULL, &rs485Task_attributes);
  wdTaskHandle = osThreadNew(StartWatchdogTask, NULL, &wdTask_attributes);
  dumpTaskHandle = osThreadNew(mbgw_dump_task, NULL, &dumpTask_attributes);
  if (tcpTaskHandle == NULL || rs485TaskHandle == NULL)
  {
    HAL_UART_Transmit(&huart1, (uint8_t*)"[RTOS] task create FAILED\r\n",
                      (uint16_t)strlen("[RTOS] task create FAILED\r\n"), 100);
  }
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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  st7789_dma_init();         /* 启动屏幕 SPI DMA */
  ui_init();                 /* 清屏 + 画标题/列表/数据 */
  for(;;)
  {
    ui_poll();               /* 摇杆上下切换从机；选中变化时重画 */
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Modbus TCP 服务器任务（上行，对电脑）
  * @param  argument  未使用
  * @retval 无（任务永不返回）
  * @note   任务壳：调用 mb_tcp 模块的服务器循环。
  */
void StartTcpTask(void *argument)
{
  mbtcp_task(argument);
}

/**
  * @brief  Modbus RTU 主站轮询任务（下行，对从机）
  * @param  argument  未使用
  * @retval 无（任务永不返回）
  * @note   任务壳：调用 mb_gateway 模块的轮询循环。
  */
void StartRs485Task(void *argument)
{
  mbgw_poll_task(argument);
}

/* ================= 看门狗 ================= */

#define WD_FEED_PERIOD_MS   500    /**< 看门狗任务检查周期（毫秒） */
#define WD_TASK_TIMEOUT_MS  3000   /**< 轮询任务心跳超过此值视为卡死（毫秒） */

/**
  * @brief  看门狗任务：检查轮询任务心跳，健康才喂独立看门狗(IWDG)
  * @param  argument  未使用
  * @retval 无（任务永不返回）
  * @note   每 WD_FEED_PERIOD_MS 检查一次轮询任务心跳：
  *         心跳新鲜 -> 刷新 IWDG；心跳超时 -> 不喂，等 IWDG 超时复位系统。
  */
void StartWatchdogTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    if ((HAL_GetTick() - mbgw_last_alive_ms()) < WD_TASK_TIMEOUT_MS)
    {
      HAL_IWDG_Refresh(&hiwdg);        /* 关键任务健康 -> 喂狗 */
    }
    /* 否则不喂狗，交给 IWDG 超时复位 */
    osDelay(WD_FEED_PERIOD_MS);
  }
}

/* ================= 栈溢出钩子 ================= */

/**
  * @brief  FreeRTOS 检测到任务栈溢出时回调（中断上下文，不能阻塞）
  * @param  xTask      溢出任务的句柄
  * @param  pcTaskName 溢出任务的名字
  * @retval 无
  * @note   直接把任务名打到 USART1，然后停机，便于定位。
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  const char *msg = "\r\n!!! STACK OVERFLOW in: ";
  (void)xTask;

  while (*msg)
  {
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = *msg++;
  }
  while (*pcTaskName)
  {
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = *pcTaskName++;
  }
  while (!(USART1->SR & USART_SR_TC)) { }
  USART1->DR = '\r';
  while (!(USART1->SR & USART_SR_TXE)) { }
  USART1->DR = '\n';

  __disable_irq();
  for (;;) { }
}

/* USER CODE END Application */

