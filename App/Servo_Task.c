/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo_Task.c
  * @brief          : Bus servo control task — sinusoidal motion
  * @note           : Drives servo #1 in a smooth sine wave pattern.
  *                   Period 4 s, amplitude ±135° (~1536 LSB).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Servo.h"
#include "Servo_Task.h"
#include <math.h>
#include "arm_config.h"
#include "Motor_DM.h"
#include "bsp_uart.h"
#include "robot_config.h"
#include "Minipc.h"
/* ========================= 全局共享状态 ========================= */

volatile ServoDisplayState servo_display = {0};

float target_yaw_angle = 0.0f;
float target_pitch_angle = 0.0f;

/* ========================= 应答回调 ========================= */

static void Servo_StateCallback(const Servo_ResponsePacket *resp)
{
    servo_display.resp_id        = resp->id;
    servo_display.resp_status    = resp->status;
    servo_display.last_resp_tick = osKernelSysTick();
    servo_display.has_response   = true;

    /* ReadPosition 应答: params[0..1] = 位置值 (大端) */
    if (resp->param_len == 2) {
        servo_display.resp_position =
            (uint16_t)(resp->params[0] << 8) | resp->params[1];
    }
}

/* ========================= 主任务 ========================= */

void Servo_Task(void const *argument)
{
    (void)argument;

    /* 注册应答回调 */
    Servo_SetResponseCallback(Servo_StateCallback);

    /* ---- 舵机初始化 ---- */
    Servo_SetMode(1, 0x01);
    osDelay(20);
    Servo_SetTorque(1, 1);
    osDelay(20);
    Servo_SetMode(2, 0x01);
    osDelay(20);
    Servo_SetTorque(2, 1);
    osDelay(20);

    for (;;)
    {
    /* ---- 摇杆速率 → 角度积分 ----
     * joystick_x / joystick_y: uint16_t, 0~4095, 中值 2048
     *   joystick_x → yaw   (SERVO_YAW,   ID=1, 左右旋转)
     *   joystick_y → pitch (SERVO_PITCH, ID=2, 俯仰)
     *
     * 摇杆偏移量控制转动速度, 归中时舵机保持在当前位置。
     * 摇杆正方向:
     *   x > 2048 = 右推 → yaw 正方向转动
     *   y < 2048 = 前推 → pitch 正方向转动
     */
    float joy_x = (float)((int16_t)(MiniPC_Data.joystick_x - 2048));
    float joy_y = (float)((int16_t)(MiniPC_Data.joystick_y - 2048));

    /* 死区: 偏差小于 ±100 强制归零, 避免中位抖动 */
    const float JOY_DEADBAND = 100.0f;
    if (fabsf(joy_x) < JOY_DEADBAND) joy_x = 0.0f;
    if (fabsf(joy_y) < JOY_DEADBAND) joy_y = 0.0f;

    if (MiniPC_Data.lost) {
        /* MiniPC 离线 → 舵机回中, 清除角度累积 */
        target_yaw_angle   = 0.0f;
        target_pitch_angle = 0.0f;
    } else {
        /* 速率积分: 角度 += 摇杆归一化 × 最大速率 × 控制周期(5ms)
         * 摇杆 ±2048 → 最大转动速度约 70 °/s */
        const float SPEED_SCALE = 70.0f;       /* 最大速率 (°/s) */
        const float DT          = 0.005f;       /* 控制周期    (s)  */
        target_yaw_angle   += (-joy_x / 2048.0f) * SPEED_SCALE * DT;
        target_pitch_angle += ( joy_y / 2048.0f) * SPEED_SCALE * DT;
    }

    VAL_LIMIT(target_pitch_angle, SERVO_PITCH_MIN, SERVO_PITCH_MAX);
    VAL_LIMIT(target_yaw_angle,   SERVO_YAW_MIN,   SERVO_YAW_MAX);

    Servo_SetPosition(SERVO_YAW,   (uint16_t)SERVO_ANGLE_TO_POS(target_yaw_angle   + SERVO_YAW_OFFSET),   10);
    Servo_SetPosition(SERVO_PITCH, (uint16_t)SERVO_ANGLE_TO_POS(target_pitch_angle + SERVO_PITCH_OFFSET), 10);
    osDelay(5);
    }
}
