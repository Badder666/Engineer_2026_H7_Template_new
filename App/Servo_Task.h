/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo_Task.h
  * @brief          : Bus servo control task — shared state for display
  * @author         : GrassFan Wang
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#include "stdint.h"
#include "stdbool.h"

#define SERVO_ANGLE_TO_POS(angle_deg)  ((uint16_t)(((int32_t)(angle_deg) + 90) * 2845 / 180))
#define SERVO_YAW   1u
#define SERVO_PITCH 2u
#define SERVO_YAW_OFFSET   -10
#define SERVO_PITCH_OFFSET  10

#define SERVO_PITCH_MAX  30
#define SERVO_PITCH_MIN -50
#define SERVO_YAW_MAX  50
#define SERVO_YAW_MIN -50
/* ======================== 共享显示状态 ======================== */
/* Servo_Task 写入 → LCD_Task 只读 */

#define SERVO_DIAG_LEN          32

extern float target_yaw_angle;
extern float target_pitch_angle;
typedef struct {
    /* --- 控制状态 (Servo_Task 写入) --- */
    uint8_t  phase;                /* 当前测试阶段 (0~7) */
    uint32_t phase_start;          /* 阶段起始 SysTick */
    uint32_t last_cmd_tick;        /* 最后发指令的 SysTick */

    /* --- 应答数据 (回调中更新) --- */
    uint8_t  resp_id;              /* 应答舵机 ID */
    uint8_t  resp_status;          /* 状态字节 */
    uint16_t resp_position;        /* 读到的位置值 */
    uint32_t last_resp_tick;       /* 最后收到应答的 SysTick */
    bool     has_response;         /* 是否有过有效应答 */

    /* --- 诊断 --- */
    char     diagnosis[SERVO_DIAG_LEN];
    uint8_t  phase_total;          /* 总阶段数 (用于显示) */
} ServoDisplayState;

/** 全局共享状态 — Servo_Task 写入, LCD_Task 只读 */
extern volatile ServoDisplayState servo_display;

/* ======================== 任务入口 ======================== */

void Servo_Task(void const *argument);

#endif /* SERVO_TASK_H */
