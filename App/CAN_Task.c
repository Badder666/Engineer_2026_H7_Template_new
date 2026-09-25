/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "Motor_DJI.h"
#include "Motor_DM.h"
#include "fdcan.h"
#include "Minipc.h"
#include "Bmi088.h"
#include "chassis_config.h"
#include "arm_config.h"
#include "PID.h"
#include <math.h>
#include <stdint.h>

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void Elevator_set(bool activated);
void Robotic_Arm_set(int part, bool activated);
void Chassis_set(bool activated);

static void Robotic_Arm_Update_Overheat(void);
static void Robotic_Arm_Motor_MIT_Or_Disable(FDCAN_TxFrame_TypeDef *frame,
                                            DM_Motor_Info_Typedef *m,
                                            const float kp, const float kd);

extern Chassis_Info_Typedef chassis_info;
extern uint8_t hand_state;

void CAN_Task(void const * argument)
{
    (void)argument;
	//DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Save_Zero_Position);
    osDelay(1);
	//DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Save_Zero_Position);

	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J6],Motor_Save_Zero_Position);
	static int robotic_arm_part = 0;
	for(;;)
    {
		Robotic_Arm_Update_Overheat();
		Robotic_Arm_set(robotic_arm_part, chassis_info.activated_flag);
		robotic_arm_part = (robotic_arm_part + 1) % 3;

        Chassis_set(chassis_info.activated_flag);

        Elevator_set(chassis_info.activated_flag);

        osDelay(1);
    }
}

void Elevator_set(const bool activated)
{
    if (activated) {
        DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Enable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Enable);
        osDelay(1);
        DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Enable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Enable);

        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[LF],Elevator_Motor[LF].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[LF].Data.Feedforward, 12);
        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[LB],Elevator_Motor[LB].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[LB].Data.Feedforward, 12);
        osDelay(1);
        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[RB],Elevator_Motor[RB].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[RB].Data.Feedforward, 12);
        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[RF],Elevator_Motor[RF].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[RF].Data.Feedforward, 12);
    }
    else {
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Disable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Disable);
        osDelay(1);
        DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Disable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Disable);
    }
}

static void Robotic_Arm_Update_Overheat(void)
{
		Robotic_Arm_Motor[0].Data.overheat = (Robotic_Arm_Motor[0].Data.Temperature_Rotor > ROBOTIC_ARM_OVERTEMP_C_DEG);
		Robotic_Arm_Motor[1].Data.overheat = (Robotic_Arm_Motor[1].Data.Temperature_Rotor > ROBOTIC_ARM_OVERTEMP_C_DEG);
		Robotic_Arm_Motor[2].Data.overheat = (Robotic_Arm_Motor[2].Data.Temperature_Rotor > ROBOTIC_ARM_OVERTEMP_C_DEG);
		Robotic_Arm_Motor[3].Data.overheat = (Robotic_Arm_Motor[3].Data.Temperature_Rotor > ROBOTIC_ARM_OVERTEMP_C_DEG);
		Robotic_Arm_Motor[4].Data.overheat = (Robotic_Arm_Motor[4].Data.Temperature_Rotor > ROBOTIC_ARM_OVERTEMP_C_DEG);
		Robotic_Arm_Motor[5].Data.overheat = (Robotic_Arm_Motor[5].Data.Temperature_Rotor > ROBOTIC_ARM_OVERTEMP_C_DEG);
}

static void Robotic_Arm_Motor_MIT_Or_Disable(FDCAN_TxFrame_TypeDef *frame,
                                            DM_Motor_Info_Typedef *m,
                                            const float kp, const float kd)
{
	if (!m->Data.overheat) {
		DM_Motor_Command(frame, m, Motor_Enable);
		//DM_Motor_CAN_TxMessage(frame, m, m->Data.Temp_Target_Position, MIT_NO_USE, kp, kd, m->Data.Feedforward, 16);
	} else {
		DM_Motor_Command(frame, m, Motor_Disable);
	}
}

#define ROBOTIC_AR_M_TORQUE_ONLY 0
void Robotic_Arm_set(const int part, const bool activated)
{
	if(activated){
		#if ROBOTIC_AR_M_TORQUE_ONLY
		if (part == 0) {
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J1], 0.0f, 0.0f);
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J2], 0.0f, 0.0f);
		} else if (part == 1) {
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J3], 0.0f, 0.0f);
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J4], 0.0f, 0.0f);
		} else if (part == 2) {
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J5], 0.0f, 0.0f);
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J6], 0.0f, 0.0f);
		}
		#else
		if (part == 0) {
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J1], 2.5f, 2.2f);
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J2], 1.5f, 41.4f);
		} else if (part == 1) {
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J3], 3.4f, 0.2f);
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J4], 0.7f, 0.02f);
		} else if (part == 2) {
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J5], 1.9f, 0.02f);
			Robotic_Arm_Motor_MIT_Or_Disable(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J6], 0.4f, 0.02f);
		}
		#endif
	}
	else {	
		if (part == 0) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Motor_Disable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J2],Motor_Disable);
		} else if (part == 1) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Motor_Disable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J4],Motor_Disable);
		} else if (part == 2) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Motor_Disable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J6],Motor_Disable);
		}
	}
}

void Chassis_set(const bool activated)
{
	if (activated) {
		M3508_motor_crt_ctrl(&hfdcan2, 0x200,
							 Chassis_Motor[LF].Data.Final_Output,
							 Chassis_Motor[LB].Data.Final_Output,
							 Chassis_Motor[RB].Data.Final_Output,
							 Chassis_Motor[RF].Data.Final_Output);
		if (hand_state == HAND_OPEN) {
			M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 3500, 0, 0, 0);
			//M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 0, 0, 0, 0);
		} else {
			M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, -3500, 0, 0, 0);
			//M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 0, 0, 0, 0);
		}
	} else {
		M3508_motor_crt_ctrl(&hfdcan2, 0x200, 0, 0, 0, 0);
		if (hand_state == HAND_CLOSE) {
			M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, -3000, 0, 0, 0);
			//M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 0, 0, 0, 0);
		} else {
		    M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 0, 0, 0, 0);
		}
	}
}
