#include "Power_Limit_communication.h"
#include <string.h>
#include "fdcan.h"

// 全局变量定义
PowerMeter_Data_t g_power_meter_data = {0};

// FDCAN发送帧结构体（用于发送清零命令）
FDCAN_TxFrame_TypeDef PowerMeter_TxFrame = {
    .hcan = &hfdcan1,
    .Header.IdType = FDCAN_STANDARD_ID,
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 1,  // 清零帧只有1字节
    .Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat = FDCAN_CLASSIC_CAN,
    .Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .Header.MessageMarker = 0,
};

/**
  * @brief  功率计初始化
  * @param  None
  * @retval None
  */
void PowerMeter_Init(void)
{
    // 初始化数据
    memset(&g_power_meter_data, 0, sizeof(PowerMeter_Data_t));

    // 配置发送帧ID
    PowerMeter_TxFrame.Header.Identifier = POWER_METER_TX_ID;
}

/**
  * @brief  更新功率计信息（在CAN接收回调中调用）
  * @param  Identifier: 指向CAN帧标识符的指针
  * @param  Rx_Buf: 指向接收数据缓冲区的指针
  * @retval None
  */
void PowerMeter_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf)
{
    // 检查标识符是否匹配
    if (*Identifier != POWER_METER_RX_ID) return;

    // 解析电压（位0-15，真实值 × 100）
    uint16_t voltage_raw = ((uint16_t)Rx_Buf[0] << 8) | (uint16_t)Rx_Buf[1];
    g_power_meter_data.voltage = (float)voltage_raw / POWER_METER_VOLTAGE_SCALE;

    // 解析电流（位16-31，真实值 × 100）
    uint16_t current_raw = ((uint16_t)Rx_Buf[2] << 8) | (uint16_t)Rx_Buf[3];
    g_power_meter_data.current = (float)current_raw / POWER_METER_CURRENT_SCALE;

    // 解析功率（位32-47，真实值 × 10）
    uint16_t power_raw = ((uint16_t)Rx_Buf[4] << 8) | (uint16_t)Rx_Buf[5];
    g_power_meter_data.power = (float)power_raw / POWER_METER_POWER_SCALE;

    // 解析能量（位48-63，真实值）
    uint16_t energy_raw = ((uint16_t)Rx_Buf[6] << 8) | (uint16_t)Rx_Buf[7];
    g_power_meter_data.energy = (float)energy_raw / POWER_METER_ENERGY_SCALE;

    // 更新状态
    g_power_meter_data.valid = true;
    g_power_meter_data.last_update_tick = HAL_GetTick();
}

/**
  * @brief  清零功率计能量
  * @param  None
  * @retval None
  */
void PowerMeter_Clear_Energy(void)
{
    // 设置发送数据
    PowerMeter_TxFrame.Data[0] = POWER_METER_CLEAR_ENERGY_CMD;

    // 发送CAN帧
    USER_FDCAN_AddMessageToTxFifoQ(&PowerMeter_TxFrame);
}

/**
  * @brief  检查功率计数据是否有效
  * @param  None
  * @retval true: 数据有效，false: 数据无效
  */
bool PowerMeter_Is_Data_Valid(void)
{
    // 检查是否有更新（500ms内收到数据视为有效）
    if (!g_power_meter_data.valid) return false;

    uint32_t current_tick = HAL_GetTick();
    if ((current_tick - g_power_meter_data.last_update_tick) > 500) {
        g_power_meter_data.valid = false;
        return false;
    }

    return true;
}

/**
  * @brief  获取电压值
  * @param  None
  * @retval 电压值 (V)
  */
float PowerMeter_Get_Voltage(void)
{
    return g_power_meter_data.voltage;
}

/**
  * @brief  获取电流值
  * @param  None
  * @retval 电流值 (A)
  */
float PowerMeter_Get_Current(void)
{
    return g_power_meter_data.current;
}

/**
  * @brief  获取功率值
  * @param  None
  * @retval 功率值 (W)
  */
float PowerMeter_Get_Power(void)
{
    return g_power_meter_data.power;
}

/**
  * @brief  获取能量值
  * @param  None
  * @retval 能量值 (J)
  */
float PowerMeter_Get_Energy(void)
{
    return g_power_meter_data.energy;
}
