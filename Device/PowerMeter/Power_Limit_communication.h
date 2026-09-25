#ifndef POWER_LIMIT_COMMUNICATION_H
#define POWER_LIMIT_COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "bsp_can.h"

// 功率计CAN帧ID定义
#define POWER_METER_RX_ID     0x306   // 反馈帧（功率计TX，主控RX）
#define POWER_METER_TX_ID     0x307   // 清零帧（功率计RX，主控TX）

// 数据缩放因子
#define POWER_METER_VOLTAGE_SCALE   100.0f  // 电压：真实值 × 100
#define POWER_METER_CURRENT_SCALE   100.0f  // 电流：真实值 × 100
#define POWER_METER_POWER_SCALE     10.0f   // 功率：真实值 × 10
#define POWER_METER_ENERGY_SCALE    1.0f    // 能量：真实值（无缩放）

// 能量清零命令
#define POWER_METER_CLEAR_ENERGY_CMD  0x01

/**
 * @brief 功率计数据结构体
 */
typedef struct {
    float voltage;      // 电压 (V)
    float current;      // 电流 (A)
    float power;        // 功率 (W)
    float energy;       // 消耗能量 (J)
    uint32_t last_update_tick;  // 上次更新时间戳
    bool valid;         // 数据是否有效
} PowerMeter_Data_t;

// 外部变量声明
extern PowerMeter_Data_t g_power_meter_data;
extern FDCAN_TxFrame_TypeDef PowerMeter_TxFrame;

// 函数声明
void PowerMeter_Init(void);
void PowerMeter_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf);
void PowerMeter_Clear_Energy(void);
bool PowerMeter_Is_Data_Valid(void);
float PowerMeter_Get_Voltage(void);
float PowerMeter_Get_Current(void);
float PowerMeter_Get_Power(void);
float PowerMeter_Get_Energy(void);

#endif // POWER_LIMIT_COMMUNICATION_H
