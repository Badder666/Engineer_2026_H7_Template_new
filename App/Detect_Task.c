/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Detect_Task.h
  * @brief          : Detect task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Detect_Task.h"
#include "Control_Task.h"
#include "Remote_Control.h"
#include "bsp_gpio.h"
#include "Bmi088.h"
#include "INS_Task.h"
#include "Minipc.h"
#include "chassis_config.h"
#include "bsp_uart.h"
#include "Motor_DM.h"
#include "PID.h"
#include "arm_config.h"
#include <stdint.h>
#include "Referee_System.h"
#include "UI.h"
#include "arm_math.h"
#include "Servo.h"
#include "Servo_Task.h"
/* USER CODE BEGIN Header_Detect_Task */
static void chassis_set_mode(Chassis_Info_Typedef* chassis);
static void chassis_ctrl_info_get(void);
static void chassis_wheel_cal(void);
static void MiniPC_Transmit_Robotic_Arm_Info(void);
static void Robotic_Arm_Ctrl_Info_Get(void);
static void Robotic_Arm_MotorPos_To_JointPos(void);
static void Robotic_Arm_MotorVel_To_JointVel(void);
static void DM_Motor_Offline_Monitor(void);

extern Chassis_Info_Typedef chassis_info;
extern float target_yaw_angle;
extern float target_pitch_angle;

static float Chassis_Direction_PID_Param[PID_PARAMETER_NUM] = {
    CHASSIS_DIRECTION_KP,
    CHASSIS_DIRECTION_KI,
    CHASSIS_DIRECTION_KD,
    CHASSIS_DIRECTION_Alpha,
    CHASSIS_DIRECTION_Deadband,
    CHASSIS_DIRECTION_LimitIntegral,
    CHASSIS_DIRECTION_LimitOutput
};
PID_Info_TypeDef Chassis_Direction_PID;

extern Referee_System_Info_TypeDef Referee_System_Info;

Hand_State_e hand_state = HAND_CLOSE;
TickType_t Detect_Task_SysTick = 0;
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
void Detect_Task(void const * argument)
{
    /* USER CODE BEGIN Detect_Task */
    (void)argument;
    PID_Init(&Chassis_Direction_PID,PID_POSITION,Chassis_Direction_PID_Param);

    /* Infinite loop */
    for(;;)
    {
        Detect_Task_SysTick = osKernelSysTick();
        Remote_Message_Moniter(&remote_ctrl);
        MiniPC_Offline_Monitor();
        MiniPC_Receive_Info();
        MiniPC_Transmit_Robotic_Arm_Info();

        chassis_set_mode(&chassis_info);
        chassis_ctrl_info_get();
        chassis_wheel_cal();

        Robotic_Arm_Ctrl_Info_Get();

        Robotic_Arm_MotorPos_To_JointPos();
        Robotic_Arm_MotorVel_To_JointVel();
        DM_Motor_Offline_Monitor();

        UI_Tick();

        float key_debug_data[12] = {
            #if 1
            MiniPC_Data.key_f,
            MiniPC_Data.key_e,
            //Robotic_Arm_Motor[J3].Data.Target_Position,
            //Robotic_Arm_Motor[J4].Data.Target_Position,
            //Robotic_Arm_Motor[J5].Data.Target_Position,
            //Robotic_Arm_Motor[J6].Data.Target_Position,
            #else
            MiniPC_Data.main_buttons,
            MiniPC_Data.handle_buttons,
            MiniPC_Data.btn2,
            remote_ctrl.rc_lost,
            #endif
        };
        USART10_Vofa_SendFloat(key_debug_data, 2);
        /* ========================================================= */

        MiniPC_Data_Update_Last();
        osDelayUntil(&Detect_Task_SysTick, 1);
    }
    /* USER CODE END Detect_Task */
}

static void chassis_set_mode(Chassis_Info_Typedef* chassis)
{
    if(chassis == NULL)
        return;

    if (remote_ctrl.rc_lost) {
        if(MiniPC_Data.key_r) HAL_NVIC_SystemReset();
        chassis->last_mode = chassis->mode;
        chassis->last_lift_mode = chassis->lift_mode;
        if(MINIPC_KEY_RISING_EDGE(key_f)) {
            if (chassis->last_mode == CHASSIS_LIFT) {
                chassis->lift_mode = LIFT_STAGE_1;
                chassis->mode = CHASSIS_AUTO_LIFT_1;
            } else if(chassis->last_mode == CHASSIS_AUTO_LIFT_1) {
                chassis->lift_mode = LIFT_STAGE_1;
                chassis->mode = CHASSIS_AUTO_LIFT_2;
            } else if(chassis->last_mode == CHASSIS_AUTO_LIFT_2) {
                chassis->lift_mode = LIFT_STAGE_3;
                chassis->mode = CHASSIS_LIFT;
            } else {
                chassis->mode = CHASSIS_DISABLE;
            }
        }else if(MINIPC_KEY_RISING_EDGE(key_q)) {
            if (chassis->mode == CHASSIS_LIFT) {
                chassis->mode = CHASSIS_DISABLE;
            } else {
                chassis->mode = CHASSIS_LIFT;
            }
        }
        if(MINIPC_KEY_RISING_EDGE(key_z)) {
            if (chassis->lift_mode == LIFT_STAGE_3) {
                chassis->lift_mode = LIFT_STAGE_5;
            } else if (chassis->lift_mode == LIFT_STAGE_5) {
                chassis->lift_mode = LIFT_STAGE_1;
            } else if (chassis->lift_mode == LIFT_STAGE_1 || chassis->lift_mode == LIFT_STAGE_6) {
                chassis->lift_mode = LIFT_STAGE_3;
            }
        }else if(MINIPC_KEY_RISING_EDGE(key_x)) {
            if(chassis->lift_mode == LIFT_STAGE_4) {
                chassis->lift_mode = LIFT_STAGE_3;
            } else {
                chassis->lift_mode = LIFT_STAGE_4;
            }
        }
        if(MINIPC_KEY_RISING_EDGE(key_e)) {
            if (chassis->gear == 0) {
                chassis->gear = 1;
            } else {
                chassis->gear = 0;
            }
        }
        if(MINIPC_KEY_RISING_EDGE(btn1)) {
            if (hand_state == HAND_OPEN) {
                hand_state = HAND_CLOSE;
            } else {
                hand_state = HAND_OPEN;
            }
        }
    } else {
        if(remote_ctrl.rc.ch[5] <= -630) HAL_NVIC_SystemReset();

        const uint16_t s1 = remote_ctrl.rc.s[1];
        const uint16_t s0 = remote_ctrl.rc.s[0];
        const uint16_t sw0 = remote_ctrl.rc.sw[0];
        const uint16_t sw1 = remote_ctrl.rc.sw[1];

        //if (switch_is_up(sw1)) {
            //hand_state = HAND_OPEN;
            //M2006_Gripper_Motor.Data.Target_Angle = GRIPPER_OPEN_POS;
        //} else {
            //hand_state = HAND_CLOSE;
            //M2006_Gripper_Motor.Data.Target_Angle = GRIPPER_CLOSE_POS;
        //}

        if (switch_is_up(sw0)) {
            chassis->last_mode = chassis->mode;
            chassis->mode = CHASSIS_DISABLE;
        } else if (remote_ctrl.rc.ch[4] >= 630) {
            chassis->last_mode = chassis->mode;
            chassis->last_lift_mode = chassis->lift_mode;
            chassis->mode = CHASSIS_LIFT;
            chassis->lift_mode = LIFT_STAGE_4;
        } else if (switch_is_up(s1) && switch_is_up(sw1)) {
            chassis->last_mode = chassis->mode;
            chassis->last_lift_mode = chassis->lift_mode;
            chassis->mode = CHASSIS_LIFT;
            switch (s0) {
                case RC_SW_DOWN: chassis->lift_mode = LIFT_STAGE_1; break;
                case RC_SW_MID:  chassis->lift_mode = LIFT_STAGE_5; break;
                case RC_SW_UP:   chassis->lift_mode = LIFT_STAGE_3; break;
                default: break;
            }
        } else if (switch_is_down(s1)) {
            chassis->last_mode = chassis->mode;
            chassis->last_lift_mode = chassis->lift_mode;
            chassis->mode = CHASSIS_AUTO_LIFT_1;
            if(chassis->last_mode == CHASSIS_LIFT) {
                chassis->lift_mode = LIFT_STAGE_1;
            }
        } else if (switch_is_down(sw1)) {
            chassis->last_mode = chassis->mode;
            chassis->last_lift_mode = chassis->lift_mode;
            chassis->mode = CHASSIS_AUTO_LIFT_2;
            if(chassis->last_mode == CHASSIS_LIFT) {
                chassis->lift_mode = LIFT_STAGE_1;
            }
        }

        if(MINIPC_KEY_RISING_EDGE(btn1)) {
            if (hand_state == HAND_OPEN) {
                hand_state = HAND_CLOSE;
            } else {
                hand_state = HAND_OPEN;
            }
        }

        if(MINIPC_KEY_RISING_EDGE(btn3)) {
            target_yaw_angle = 0.0f;
            target_pitch_angle = 0.0f;
        }

    }
}

static void chassis_ctrl_info_get(void)
{
    if (remote_ctrl.rc_lost) {
        if(chassis_info.mode == CHASSIS_LIFT){
            chassis_info.target_vx = ((MiniPC_Data.key_w-MiniPC_Data.key_s) * 0.15f) * (1+MiniPC_Data.key_shift) * (1+1.5*chassis_info.gear);
            chassis_info.target_vy = ((MiniPC_Data.key_a-MiniPC_Data.key_d) * 0.15f) * (1+MiniPC_Data.key_shift) * (1+1.5*chassis_info.gear);
            chassis_info.target_vw = (float)MiniPC_Data.mouse_x * -0.015f + (MiniPC_Data.key_c - MiniPC_Data.key_v) * 0.3f;

            /* 主动旋转时禁用方向锁定, 让目标方向跟随当前 yaw, 避免松杆瞬间残留误差爆发。
             * 仅用 C/V 时 mouse_x 为 0, 若仍走 else 会把 Chassis_Direction_PID 叠到 target_vw 上, 与按键角速度对冲。 */
            if (MiniPC_Data.mouse_x != 0 || MiniPC_Data.key_c != MiniPC_Data.key_v) {
                chassis_info.target_direction = INS_Info.Yaw_Angle;
                Chassis_Direction_PID.PID_Calc_Clear(&Chassis_Direction_PID);
            } else {
                Single_Angle_PID_Calculate(&Chassis_Direction_PID, chassis_info.target_direction, INS_Info.Yaw_Angle);
                chassis_info.target_vw += Chassis_Direction_PID.Output;
            }
            chassis_info.target_direction = F_Loop_Constrain(chassis_info.target_direction, -180.0f, 180.0f);
        }
    } else {
        RC_CH_APPLY_DEADBAND(remote_ctrl.rc.ch[0]);
        RC_CH_APPLY_DEADBAND(remote_ctrl.rc.ch[2]);
        RC_CH_APPLY_DEADBAND(remote_ctrl.rc.ch[3]);

        if(chassis_info.mode == CHASSIS_LIFT){
            chassis_info.target_vx = (float)remote_ctrl.rc.ch[3] * RC_TO_VX;
            chassis_info.target_vy = (float)remote_ctrl.rc.ch[2] * RC_TO_VY;
            chassis_info.target_vw = (float)remote_ctrl.rc.ch[0] * RC_TO_VW;

            /* 用户主动旋转时禁用方向锁定, 让目标方向跟随当前 yaw, 避免松杆瞬间残留误差爆发 */
            if (remote_ctrl.rc.ch[0] != 0) {
                chassis_info.target_direction = INS_Info.Yaw_Angle;
                Chassis_Direction_PID.PID_Calc_Clear(&Chassis_Direction_PID);
            } else {
                Single_Angle_PID_Calculate(&Chassis_Direction_PID, chassis_info.target_direction, INS_Info.Yaw_Angle);
                chassis_info.target_vw += Chassis_Direction_PID.Output;
            }
            chassis_info.target_direction = F_Loop_Constrain(chassis_info.target_direction, -180.0f, 180.0f);
        }
    }
}

static void chassis_wheel_cal(void)
{
    const float vx = chassis_info.target_vx;
    const float vy = chassis_info.target_vy;
    const float vw = chassis_info.target_vw;

    Chassis_Motor[LF].Data.Target_Velocity =  (vx - vy - vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    Chassis_Motor[LB].Data.Target_Velocity =  (vx + vy - vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    Chassis_Motor[RB].Data.Target_Velocity = -(vx - vy + vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    Chassis_Motor[RF].Data.Target_Velocity = -(vx + vy + vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
}

static void MiniPC_Transmit_Robotic_Arm_Info(void)
{
    /* 电机空间 → 关节空间逆转换（与 arm_ctrl_info_get 中的正转换互逆） */
    float J1_Pos = Robotic_Arm_Motor[J1].Data.Position;
    float J2_Pos = Robotic_Arm_Motor[J2].Data.Position + PI - 0.07f;
    float J3_Pos = Robotic_Arm_Motor[J3].Data.Position / 1.65f - PI - 0.05f;
    float J4_Pos = -Robotic_Arm_Motor[J4].Data.Position;
    float J5_Pos = Robotic_Arm_Motor[J5].Data.Position;
    float J6_Pos = -Robotic_Arm_Motor[J6].Data.Position;

    float Robotic_Arm_Info[6] = {J1_Pos, J2_Pos, J3_Pos, J4_Pos, J5_Pos, J6_Pos};
    MiniPC_Transmit_Info(Robotic_Arm_Info, 6);
}

static void Robotic_Arm_Ctrl_Info_Get(void)
{
    Robotic_Arm_Motor[J1].Data.Target_Position = MiniPC_Data.joint_pos_data[J1];
    Robotic_Arm_Motor[J2].Data.Target_Position = MiniPC_Data.joint_pos_data[J2]-PI+0.07f;
    Robotic_Arm_Motor[J3].Data.Target_Position = (PI+MiniPC_Data.joint_pos_data[J3]+0.05f)*1.65f;
    Robotic_Arm_Motor[J4].Data.Target_Position = -MiniPC_Data.joint_pos_data[J4];
    Robotic_Arm_Motor[J5].Data.Target_Position = MiniPC_Data.joint_pos_data[J5];
    Robotic_Arm_Motor[J6].Data.Target_Position = -MiniPC_Data.joint_pos_data[J6];

    #if 0
    Robotic_Arm_Motor[J1].Data.Feedforward = MiniPC_Data.joint_feedforword_data[J1];
    Robotic_Arm_Motor[J2].Data.Feedforward = MiniPC_Data.joint_feedforword_data[J2];
    Robotic_Arm_Motor[J3].Data.Feedforward = MiniPC_Data.joint_feedforword_data[J3]/1.65f;
    Robotic_Arm_Motor[J4].Data.Feedforward = MiniPC_Data.joint_feedforword_data[J4];
    Robotic_Arm_Motor[J5].Data.Feedforward = MiniPC_Data.joint_feedforword_data[J5];
    Robotic_Arm_Motor[J6].Data.Feedforward = MiniPC_Data.joint_feedforword_data[J6];
    #endif
}

/**
 * @brief  电机空间 → 关节空间转换
 * @note   将 DM 电机原始位置转换为机械臂关节角度
 */
static void Robotic_Arm_MotorPos_To_JointPos(void)
{
    Robotic_Arm_Motor[J1].Data.Joint_Position =  Robotic_Arm_Motor[J1].Data.Position;
    Robotic_Arm_Motor[J2].Data.Joint_Position =  Robotic_Arm_Motor[J2].Data.Position + PI - 0.07f;
    Robotic_Arm_Motor[J3].Data.Joint_Position =  Robotic_Arm_Motor[J3].Data.Position / 1.65f - PI - 0.05f;
    Robotic_Arm_Motor[J4].Data.Joint_Position = -Robotic_Arm_Motor[J4].Data.Position;
    Robotic_Arm_Motor[J5].Data.Joint_Position =  Robotic_Arm_Motor[J5].Data.Position;
    Robotic_Arm_Motor[J6].Data.Joint_Position = -Robotic_Arm_Motor[J6].Data.Position;
}

/**
 * @brief  电机速度 → 关节速度转换
 * @note   对 MotorPos → JointPos 转换求导得到速度映射关系
 */
static void Robotic_Arm_MotorVel_To_JointVel(void)
{
    Robotic_Arm_Motor[J1].Data.Joint_Velocity =  Robotic_Arm_Motor[J1].Data.Velocity;
    Robotic_Arm_Motor[J2].Data.Joint_Velocity =  Robotic_Arm_Motor[J2].Data.Velocity;
    Robotic_Arm_Motor[J3].Data.Joint_Velocity =  Robotic_Arm_Motor[J3].Data.Velocity / 1.65f;
    Robotic_Arm_Motor[J4].Data.Joint_Velocity = -Robotic_Arm_Motor[J4].Data.Velocity;
    Robotic_Arm_Motor[J5].Data.Joint_Velocity =  Robotic_Arm_Motor[J5].Data.Velocity;
    Robotic_Arm_Motor[J6].Data.Joint_Velocity = -Robotic_Arm_Motor[J6].Data.Velocity;
}

/**
 * @brief  DM 电机离线检测
 * @note   每次主循环递减 online_cnt，若连续收不到 CAN 帧则置 offline = true
 *         online_cnt 由 DM_Motor_Info_Update 在每个 CAN RX 中断中重置为 0xFA
 *         阈值 0x32 对应约 200 ms 无数据即为离线（1ms 循环）
 */
static void DM_Motor_Offline_Monitor(void)
{
    for (uint8_t i = J1; i <= J6; i++)
    {
        if (Robotic_Arm_Motor[i].Data.online_cnt <= 0x32U)
        {
            Robotic_Arm_Motor[i].Data.offline = true;
        }
        else if (Robotic_Arm_Motor[i].Data.online_cnt > 0)
        {
            Robotic_Arm_Motor[i].Data.online_cnt--;
        }
    }
}
