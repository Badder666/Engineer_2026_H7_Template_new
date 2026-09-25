/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : chassis_config.h
  * @brief          : Configuration parameters for the robot chassis
  * @author         : Wang Xinyuan
  * @date           : 2026/03/07
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

/* Exported constants --------------------------------------------------------*/

/** * @brief Chassis wheel order definition*/
#define LF 0u
#define LB 1u
#define RB 2u
#define RF 3u

/** * @brief Remote control channel max value*/
#define MAX_RC_CH_VALUE 660.0f

/** * @brief Remote control channel deadband) */
#define RC_CH_DEADBAND 6
#define RC_CH_APPLY_DEADBAND(ch) do { \
    if ((ch) <= RC_CH_DEADBAND && (ch) >= -RC_CH_DEADBAND) { (ch) = 0; } \
} while (0U)

#define MAX_CHASSIS_VX_SPEED 1.8f
#define MAX_CHASSIS_VY_SPEED 1.8f
#define MAX_CHASSIS_VW_SPEED 1.5f

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/MAX_RC_CH_VALUE)
#define RC_TO_VY  (MAX_CHASSIS_VY_SPEED/-MAX_RC_CH_VALUE)
#define RC_TO_VW  (MAX_CHASSIS_VW_SPEED/-MAX_RC_CH_VALUE)    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

#define ROTATE_RATIO 0.6f
#define WHEEL_RPM_RATIO 2200.0f
#define CHASSIS_FF_SPEED_COEF 9.0f

#define CHASSIS_TARGET_VELOCITY_RAMP 16.0f

#define CHASSIS_KP 11.0f
#define CHASSIS_KI 0.2f
#define CHASSIS_KD 70.0f
#define CHASSIS_Alpha 0.6f
#define CHASSIS_Deadband 0.0f
#define CHASSIS_LimitIntegral 4000.0f
#define CHASSIS_LimitOutput 15000.0f

/* @brief Chassis Direction PID (Yaw Angle Loop) parameters */
#define CHASSIS_DIRECTION_KP 0.1f
#define CHASSIS_DIRECTION_KI 0.0001f
#define CHASSIS_DIRECTION_KD 5.0f
#define CHASSIS_DIRECTION_Alpha 0.5f
#define CHASSIS_DIRECTION_Deadband 0.0f
#define CHASSIS_DIRECTION_LimitIntegral 2000.0f
#define CHASSIS_DIRECTION_LimitOutput 3.0f
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* Exported macros -----------------------------------------------------------*/

/**
  * @brief Limit value macro
  */
#ifndef VAL_LIMIT
#define VAL_LIMIT(x, min, max) do { \
    if ((x) > (max)) { (x) = (max); } \
    else if ((x) < (min)) { (x) = (min); } \
} while (0U)
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    CHASSIS_DISABLE,
    CHASSIS_LIFT,
    CHASSIS_AUTO_LIFT_1,
    CHASSIS_AUTO_LIFT_2,
} Chassis_Mode_e;

typedef enum
{
    LIFT_STAGE_1,
    LIFT_STAGE_2,
    LIFT_STAGE_3,
    LIFT_STAGE_4, //prepare to land
    LIFT_STAGE_5, //to get the prop
    LIFT_STAGE_6,
    LIFT_STAGE_7,
} Chassis_LIFT_Mode_e;

typedef struct
{
    Chassis_Mode_e mode;
    Chassis_Mode_e last_mode;
    Chassis_LIFT_Mode_e lift_mode;
    Chassis_LIFT_Mode_e last_lift_mode;

    float target_vx;
    float target_vy;
    float target_vw;

    float target_direction;

    bool activated_flag;
    uint16_t lift_counter_1;
    bool countering_1;
    uint16_t lift_counter_2;
    bool countering_2;

    uint8_t gear;
    uint8_t last_gear;
} Chassis_Info_Typedef;

/* @brief Elevator (DM6006) parameters*/
#define MIT_NO_USE 0u
#define LIFTING_TIME 1000u //ms
#define ELEVATOR_KP 2.f
#define ELEVATOR_FEEDFORWARD_FOR_LB_RF 1.27f
#define ELEVATOR_FEEDFORWARD_FOR_LF_RB -1.27f

#define ELEVATOR_USUAL_POS 0.f

#define ELEVATOR_LF_1st_ACTIVATED_POS -9.44f
#define ELEVATOR_LB_1st_ACTIVATED_POS 9.37f
#define ELEVATOR_RB_1st_ACTIVATED_POS -9.40f
#define ELEVATOR_RF_1st_ACTIVATED_POS 9.40f

#define ELEVATOR_LF_2nd_ACTIVATED_POS (ELEVATOR_LF_1st_ACTIVATED_POS*1.0f)
#define ELEVATOR_LB_2nd_ACTIVATED_POS (ELEVATOR_LB_1st_ACTIVATED_POS*0.1f)
#define ELEVATOR_RB_2nd_ACTIVATED_POS (ELEVATOR_RB_1st_ACTIVATED_POS*0.1f)
#define ELEVATOR_RF_2nd_ACTIVATED_POS (ELEVATOR_RF_1st_ACTIVATED_POS*1.0f)

#define ELEVATOR_LF_3rd_ACTIVATED_POS (ELEVATOR_LF_1st_ACTIVATED_POS*0.65f)
#define ELEVATOR_LB_3rd_ACTIVATED_POS (ELEVATOR_LB_1st_ACTIVATED_POS*0.65f)
#define ELEVATOR_RB_3rd_ACTIVATED_POS (ELEVATOR_RB_1st_ACTIVATED_POS*0.65f)
#define ELEVATOR_RF_3rd_ACTIVATED_POS (ELEVATOR_RF_1st_ACTIVATED_POS*0.65f)

#define ELEVATOR_LF_4th_ACTIVATED_POS (ELEVATOR_LF_1st_ACTIVATED_POS*0.2f)
#define ELEVATOR_LB_4th_ACTIVATED_POS (ELEVATOR_USUAL_POS)
#define ELEVATOR_RB_4th_ACTIVATED_POS (ELEVATOR_USUAL_POS)
#define ELEVATOR_RF_4th_ACTIVATED_POS (ELEVATOR_RF_1st_ACTIVATED_POS*0.2f)
/* @brief Chassis auto-lifting parameters */
#define CHASSIS_AUTO_LIFT_TARGET_VELOCITY       650     // 自动抬升时底盘电机目标转速 (rpm)
#define CHASSIS_AUTO_LIFT_STALL_VELOCITY_TH     80      // 判定电机堵转的速度阈值 (rpm)
#define CHASSIS_AUTO_LIFT_STAGE1_COUNTER_TH     100     // 阶段1->阶段2 堵转计数阈值 (ms)
#define CHASSIS_AUTO_LIFT_STAGE2_COUNTER_TH     300     // 阶段2->阶段3 堵转计数阈值 (ms)

#endif /* CHASSIS_CONFIG_H */
