/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : GravityCompensation.h
  * @brief          : 6-DOF 机械臂重力补偿前馈力矩计算
  *                   基于改良(经典)DH参数 + τ_g = -Σ ((r × F_g) · z_j)
  *                   使用 CMSIS-DSP arm_math 进行 4x4 变换矩阵链式乘法
  * @author         : Ported from Python (Chen XingYu / Lu Yaoheng)
  * @date           : 2026/04/26
  * @version        : v1.0
  ******************************************************************************
  * @attention
  * 1) 输入: 6 个关节角(rad)
  * 2) 输出: 6 个关节的重力补偿前馈力矩(N·m), 顺序与关节 1..6 对齐
  * 3) 在 GravityComp_Init() 中根据 Python 工程数据完成模型初始化, 与
  *    robot_gravity_compensation.py 中的 RobotModel 完全一致.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "robot_config.h"
#include "arm_math.h"

/* Exported macros -----------------------------------------------------------*/
/**
 * @brief 机械臂自由度
 */
#define GRAV_COMP_DOF       6U

/**
 * @brief 6-DOF 机械臂重力补偿模型与运行时数据
 *
 * @note  改良DH参数(modified-DH)与各连杆质心定义和原 Python 工程一致:
 *        - alpha[i], a[i], d[i], theta_offset[i] : 关节 i+1 的 DH 参数 (rad/m)
 *        - com_local[i][3] : 第 i+1 个连杆质心, 在父坐标系 i 中表达 (m)
 *          其中关节 1 的父坐标系为世界坐标系
 *        - masses[i] : 第 i+1 个连杆的质量 (kg)
 *        - gravity[3] : 世界坐标系下的重力加速度向量 (m/s^2)
 */
typedef struct
{
    /* ---- DH 参数 (rad / m) ---- */
    float alpha[GRAV_COMP_DOF];        /*!< 连杆扭转角 (rad) */
    float a[GRAV_COMP_DOF];            /*!< 连杆长度   (m)   */
    float d[GRAV_COMP_DOF];            /*!< 连杆偏置   (m)   */
    float theta_offset[GRAV_COMP_DOF]; /*!< 关节零位偏置(rad) */

    /* ---- 动力学参数 ---- */
    float com_local[GRAV_COMP_DOF][3]; /*!< 各连杆质心 (父系下) (m) */
    float masses[GRAV_COMP_DOF];       /*!< 各连杆质量 (kg) */
    float gravity[3];                  /*!< 世界系重力向量 (m/s^2) */

    /* ---- 运行时数据 ---- */
    float current_theta[GRAV_COMP_DOF];      /*!< 最近一次输入的关节角 (rad) */
    float feedforward_torque[GRAV_COMP_DOF]; /*!< 重力补偿前馈力矩 (N·m), 与关节 1..6 对齐 */

} GravityComp_Info_TypeDef;

/* Exported variables --------------------------------------------------------*/
/**
 * @brief 全局重力补偿实例
 */
extern GravityComp_Info_TypeDef gGravityComp;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  使用与 Python 工程相同的 DH/质量/质心参数初始化全局模型.
  * @retval None
  */
extern void GravityComp_Init(void);

/**
  * @brief  输入 6 个关节角(rad), 计算重力补偿前馈力矩(N·m), 写入用户缓冲区,
  *         同时刷新全局实例 gGravityComp.feedforward_torque.
  * @param  theta:        长度为 6 的关节角数组 (rad), 与关节 1..6 对应.
  * @param  torques_out:  长度为 6 的输出缓冲, 与关节 1..6 对应.
  * @retval None
  */
extern void GravityComp_Compute(const float theta[GRAV_COMP_DOF],
                                float torques_out[GRAV_COMP_DOF]);

/**
  * @brief  仅刷新全局实例的 feedforward_torque (前馈数值列表), 不需要外部缓冲.
  *         典型用法: 控制环里把六个关节角传进来, 之后从
  *         gGravityComp.feedforward_torque[i] 读取每个关节的重力前馈.
  * @param  theta: 长度为 6 的关节角数组 (rad).
  * @retval None
  */
extern void GravityComp_UpdateFeedforward(const float theta[GRAV_COMP_DOF]);

/**
  * @brief  获取重力补偿前馈数组的只读指针 (长度 = GRAV_COMP_DOF).
  * @retval const float* 指向 gGravityComp.feedforward_torque 的指针.
  */
extern const float *GravityComp_GetFeedforward(void);

/**
  * @brief  设置某个连杆的质量(kg). 当机械臂末端工具/负载变化时使用.
  * @param  link_idx: 连杆索引 0..5, 对应关节 1..6.
  * @param  mass:     新的质量值 (kg).
  * @retval None
  */
extern void GravityComp_SetMass(uint8_t link_idx, float mass);

/**
  * @brief  设置某个连杆的质心位置(在父坐标系下) (m).
  * @param  link_idx: 连杆索引 0..5, 对应关节 1..6.
  * @param  x,y,z:    父系下的质心坐标 (m).
  * @retval None
  */
extern void GravityComp_SetCoM(uint8_t link_idx, float x, float y, float z);

/**
  * @brief  设置世界坐标系下的重力向量(m/s^2). 默认为 (0, 0, -GravityAccel).
  * @param  gx,gy,gz: 重力分量 (m/s^2).
  * @retval None
  */
extern void GravityComp_SetGravity(float gx, float gy, float gz);

#ifdef __cplusplus
}
#endif

#endif /* GRAVITY_COMPENSATION_H */
