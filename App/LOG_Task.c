/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LOG_Task.c
  * @brief          : ESP32 log demo task — send state data via UART to ESP32
  * @author         :
  * @date           : 2026/07/24
  * @version        : v1.0
  ******************************************************************************
  * @attention      :
  *  通过 UART10 向 ESP32 异步发送机械臂状态数据。
  *  使用 ESP32_LOG 模块的 FreeRTOS 队列 + 独立发送任务，不阻塞控制循环。
  *
  *  用法:
  *    1. 确保 huart10 已初始化
  *    2. 在 MX_FREERTOS_Init 中创建本任务 (已在 freertos.c 中添加)
  *    3. 上位机按以下帧格式解析:
  *       帧头(0xA5 0x5A) + 版本(1) + 类型(1) + 序号(1) +
  *       name长度(1) + data长度(2) + name(N) + data(M) + CRC16(2)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "ESP32_LOG.h"
#include "Motor_DM.h"           /* Robotic_Arm_Motor, Elevator_Motor */
#include "chassis_config.h"     /* chassis_info */
#include "Minipc.h"             /* MiniPC_Data */
#include <string.h>
#include "bsp_uart.h"

/* Private define ------------------------------------------------------------*/
#define LOG_TASK_INTERVAL_MS    10U    /* 10ms 发一次, 100Hz */

/* Private variables ---------------------------------------------------------*/
static uint32_t log_cycle = 0U;

/* ---------------------------------------------------------------------------
 * 发送一组简单的状态文本，便于检查 ESP32_LOG 是否正常工作
 * ---------------------------------------------------------------------------*/
static void LOG_Demo_Text(void)
{
    /* 发送一条纯文本 */
    Uart_State_Send_Text("status", "running");

    /* 发送 MiniPC 在线状态 */
    if (MiniPC_Data.lost)
    {
        Uart_State_Send_Text("minipc", "offline");
    }
    else
    {
        Uart_State_Send_Text("minipc", "online");
    }
}

/* ---------------------------------------------------------------------------
 * 发送机械臂 J3 关节的数值 (位置、目标位置、速度)
 * ---------------------------------------------------------------------------*/
static void LOG_Robotic_Arm_J3(void)
{
    Uart_State_Send_Float("J3_pos", Robotic_Arm_Motor[2].Data.Position);
    Uart_State_Send_Float("J3_tar", Robotic_Arm_Motor[2].Data.Target_Position);
    Uart_State_Send_Float("J3_vel", Robotic_Arm_Motor[2].Data.Velocity);
}

/* ---------------------------------------------------------------------------
 * 发送计数器和运行时间 (调试用)
 * ---------------------------------------------------------------------------*/
static void LOG_Cycle(void)
{
    Uart_State_Send_Uint32("cycle", log_cycle);
}

/* ---------------------------------------------------------------------------
 * LOG_Task 主函数
 *   前提: ESP32_LOG 队列 + 发送任务已在 MCU_Init() 中创建
 *   功能: 周期发送 demo 数据
 * ---------------------------------------------------------------------------*/
void LOG_Task(void const * argument)
{
    (void)argument;
    /* 主循环：周期发送 demo 数据 */
    for (;;)
    {
        log_cycle++;

        //LOG_Cycle();
        //LOG_Demo_Text();
        //LOG_Robotic_Arm_J3();

        osDelay(LOG_TASK_INTERVAL_MS);
    }
}
