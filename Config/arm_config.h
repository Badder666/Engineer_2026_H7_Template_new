/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : arm_config.h
  * @brief          : Basic configuration for robotic arm
  *                   机械臂基础配置参数定义头文件
  * @author         : Wang Xinyuan
  * @date           : 2026/03/12
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOTIC_ARM_CONFIG_H
#define ROBOTIC_ARM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "arm_math.h"

/* Exported constants --------------------------------------------------------*/
#define ROBOTIC_ARM_DOF 6u

/** 机械臂关节电机软件过温保护阈值 (°C)，超过则失能对应电机 */
#define ROBOTIC_ARM_OVERTEMP_C_DEG 90.0f

#define J1 0u
#define J2 1u
#define J3 2u
#define J4 3u
#define J5 4u
#define J6 5u

typedef enum
{
  HAND_OPEN = 0U,
  HAND_CLOSE = 1U,
} Hand_State_e;

#define J1_INITIAL_POS 0.00f
#define J2_INITIAL_POS 2.44f
#define J3_INITIAL_POS -2.44f
#define J4_INITIAL_POS 0.00f
#define J5_INITIAL_POS 0.00f
#define J6_INITIAL_POS 0.00f 

/** 每个 1 ms 控制周期允许关节电机目标位置变化的最大值 (rad) */
#define ROBOTIC_ARM_TARGET_POSITION_RAMP 0.001f

#define GRIPPER_OPEN_POS -21.0f
#define GRIPPER_CLOSE_POS 133.8f

#define GRIPPER_KP 50.5f
#define GRIPPER_KI 0.1f
#define GRIPPER_KD 30.5f
#define GRIPPER_Alpha 0.9f
#define GRIPPER_Deadband 1.5f
#define GRIPPER_LimitIntegral 3000u
#define GRIPPER_LimitOutput 3000u

/* ----------------------------------------------------------------------------
 * 机械臂关节 MIT 前馈项 PID(用于消除关节稳态误差)
 *  - 输入: Target_Position - Position (单位: rad)
 *  - 输出: 直接作为 MIT 模式 Feedforward 力矩 (单位: N·m)
 *  - 注意: J3/J4/J6 电机正转方向与关节正转方向相反, 使用时需对前馈值取反
 *  - 注意: PID.c 中 LimitIntegral 限的是累加误差 Integral 本身, Iout = KI * Integral
 *          所以为了让 Iout 能达到 LimitOutput, 需保证 KI * LimitIntegral >= LimitOutput
 *
 *  分组:
 *      Group J1 (基座回转, 单独一组): J1
 *      Group A  (大关节, 靠近基座, 承重大): J2, J3
 *      Group B  (小关节, 末端三轴, 负载小): J4, J5, J6
 * -------------------------------------------------------------------------- */

/* ---------- Group J1: 基座回转关节 (单独调参) ---------- */
#define ROBOTIC_ARM_FF_J1_KP              0.15f
#define ROBOTIC_ARM_FF_J1_KI              0.01f
#define ROBOTIC_ARM_FF_J1_KD              2.7f
#define ROBOTIC_ARM_FF_J1_Alpha           0.5f
#define ROBOTIC_ARM_FF_J1_Deadband        0.00f
#define ROBOTIC_ARM_FF_J1_LimitIntegral   9400.0f     /* 单位: rad·tick (累加误差) */
#define ROBOTIC_ARM_FF_J1_LimitOutput     2.0f       /* N·m */

/* ---------- Group A: J2 / J3 (肩/肘关节) ---------- */
#define ROBOTIC_ARM_FF_A_KP              0.25f
#define ROBOTIC_ARM_FF_A_KI              0.01f
#define ROBOTIC_ARM_FF_A_KD              22.7f
#define ROBOTIC_ARM_FF_A_Alpha           0.5f
#define ROBOTIC_ARM_FF_A_Deadband        0.00f
#define ROBOTIC_ARM_FF_A_LimitIntegral   9400.0f     /* 单位: rad·tick (累加误差) */
#define ROBOTIC_ARM_FF_A_LimitOutput     5.0f       /* N·m */

/* ---------- Group B: J4 / J5 / J6 (末端三轴) ---------- */
#define ROBOTIC_ARM_FF_B_KP              0.14f
#define ROBOTIC_ARM_FF_B_KI              0.02f
#define ROBOTIC_ARM_FF_B_KD              0.03f
#define ROBOTIC_ARM_FF_B_Alpha           0.5f
#define ROBOTIC_ARM_FF_B_Deadband        0.00f      /* rad, 约 1.15° */
#define ROBOTIC_ARM_FF_B_LimitIntegral   100.0f      /* 单位: rad·tick (累加误差) */
#define ROBOTIC_ARM_FF_B_LimitOutput     2.0f       /* N·m */

/* ----------------------------------------------------------------------------
 * 2-DOF RR 动力学补偿参数 (J2=肩关节, J3=肘关节)
 * 经典平面 RR 构型, 忽略加速度项:
 *   τ₁ = -m₂·l₁·l₂·sin(q₂)·(dq₂² + 2·dq₁·dq₂)  + m₂·g·l₂·cos(q₁+q₂)
 *        + (m₁+m₂)·g·l₁·cos(q₁)
 *   τ₂ = +m₂·l₁·l₂·sin(q₂)·dq₁²                   + m₂·g·l₂·cos(q₁+q₂)
 * 用户自行标定调参                                                    */
#define RR_M1  2.0f    /* 连杆 1 质量 (kg) */
#define RR_M2  2.0f    /* 连杆 2 质量 (kg) */
#define RR_L1  0.32f    /* 连杆 1 长度 (m) */
#define RR_L2  0.18f    /* 连杆 2 长度 (m) */

#endif /* ROBOTIC_ARM_CONFIG_H */

