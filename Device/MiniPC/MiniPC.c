/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions
  * @author         : GarssFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Minipc.h"
#include "usbd_cdc_if.h"
#include "arm_config.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef union {
    float f;
    uint8_t bytes[4];
} FloatConverter;

/* Private function prototypes -----------------------------------------------*/
static float bytes_to_float_union(const uint8_t *buffer);
static void  float_to_bytes_union(float value, uint8_t *buffer);

/* Public variables ----------------------------------------------------------*/
/* MiniPC 数据全局实例 */
MiniPC_DataTypeDef MiniPC_Data = {
    .joint_pos_data = {J1_INITIAL_POS, J2_INITIAL_POS, J3_INITIAL_POS,
                   J4_INITIAL_POS, J5_INITIAL_POS, J6_INITIAL_POS},
    .main_buttons = 0, .handle_buttons = 0,
    .btn1 = 0, .btn2 = 0, .btn3 = 0, .btn4 = 0,
    .joystick_x = 2048, .joystick_y = 2048,
    .mouse_x = 0, .mouse_y = 0, .mouse_z = 0,
    .mouse_left = 0, .mouse_right = 0, .mouse_mid = 0,
    .key_w = 0, .key_s = 0, .key_a = 0, .key_d = 0,
    .key_shift = 0, .key_ctrl = 0, .key_q = 0, .key_e = 0,
    .key_r = 0, .key_f = 0, .key_g = 0, .key_z = 0,
    .key_x = 0, .key_c = 0, .key_v = 0, .key_b = 0,
    .online_cnt = 0, .lost = true
};

/* MiniPC 数据上一循环状态镜像 - 用于边沿/跳变检测 */
MiniPC_DataTypeDef MiniPC_Data_Last = {
    .joint_pos_data = {J1_INITIAL_POS, J2_INITIAL_POS, J3_INITIAL_POS,
                   J4_INITIAL_POS, J5_INITIAL_POS, J6_INITIAL_POS},
    .main_buttons = 0, .handle_buttons = 0,
    .btn1 = 0, .btn2 = 0, .btn3 = 0, .btn4 = 0,
    .joystick_x = 0, .joystick_y = 0,
    .mouse_x = 0, .mouse_y = 0, .mouse_z = 0,
    .mouse_left = 0, .mouse_right = 0, .mouse_mid = 0,
    .key_w = 0, .key_s = 0, .key_a = 0, .key_d = 0,
    .key_shift = 0, .key_ctrl = 0, .key_q = 0, .key_e = 0,
    .key_r = 0, .key_f = 0, .key_g = 0, .key_z = 0,
    .key_x = 0, .key_c = 0, .key_v = 0, .key_b = 0,
    .online_cnt = 0, .lost = true
};

/* Private variables ---------------------------------------------------------*/
/* 发送缓冲：帧头(1) + 6*float(24) + 校验(1) + 帧尾(1) = 27 字节足够 */
static uint8_t joint_data_transmit[27] = {0};

/* Private functions ---------------------------------------------------------*/
static float bytes_to_float_union(const uint8_t *buffer)
{
    FloatConverter converter;
    for (int i = 0; i < 4; i++) {
        converter.bytes[i] = buffer[i];
    }
    return converter.f;
}

static void float_to_bytes_union(float value, uint8_t *buffer)
{
    FloatConverter converter;
    converter.f = value;
    for (int i = 0; i < 4; i++) {
        buffer[i] = converter.bytes[i];
    }
}

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  通过 USB CDC 向上位机发送 float 数组
  * @param  Buf: 待发送的 float 数据指针
  * @param  Len: float 数据个数
  * @retval CDC_Transmit_HS 的返回值
  */
uint8_t MiniPC_Transmit_Info(float* Buf, uint16_t Len)
{
    /* 校验位置零，覆盖上一次校验位 */
    joint_data_transmit[Len * 4 + 1] = 0;
    joint_data_transmit[0] = 0xAA;
    for (uint8_t i = 0; i < Len; i++) {
        float_to_bytes_union(Buf[i], &joint_data_transmit[1 + i * 4]);
    }
    for (uint8_t i = 1; i < Len * 4 + 1; i++) {
        joint_data_transmit[Len * 4 + 1] += joint_data_transmit[i];
    }
    joint_data_transmit[Len * 4 + 2] = 0x55;
    return CDC_Transmit_HS(joint_data_transmit, Len * 4 + 3);
}

/**
  * @brief  解析从小电脑接收的 USB 虚拟串口数据
  * @note   数据包格式：帧头(0xAA) + 数据(57字节) + 校验和(1字节) + 帧尾(0x55)
  *         数据布局：6个float(24B) + 自定义控制器(6B) + 3个int16_t(6B) + 21个uint8_t(21B)
  *         总长度：60字节
  * @retval None - 解析结果存入全局变量 MiniPC_Data
  */
void MiniPC_Receive_Info(void)
{
    /* 数据部分大小：6*4 + 6 + 3*2 + 21*1 = 57字节 */
    const uint32_t data_len = 57;
    /* 完整数据包：帧头(1) + 数据(57) + 校验(1) + 帧尾(1) = 60字节 */
    const uint32_t packet_len = data_len + 3;

    /* 检查是否有新数据到达 */
    if (USB_RxReady == 0) {
        return;
    }
    USB_RxReady = 0;

    if (USB_RxLength < packet_len) {
        return;
    }

    /* 帧头帧尾检查 */
    if (UserRxBufferHS[0] != 0xAA ||
        UserRxBufferHS[packet_len - 1] != 0x55) {
        return;
    }

    /* 校验和计算：累加所有数据字节 */
    uint8_t verification = 0;
    for (uint32_t i = 1; i <= data_len; i++) {
        verification += UserRxBufferHS[i];
    }
    if (UserRxBufferHS[packet_len - 2] != verification) {
        return;
    }

    uint8_t* pbuf = &UserRxBufferHS[1];
    uint32_t idx = 0;

    /* 解析6个float - 关节数据 (偏移 0-23) */
    for (uint32_t i = 0; i < 6; i++) {
        MiniPC_Data.joint_pos_data[i] = bytes_to_float_union(&pbuf[idx]);
        idx += 4;
    }

    /* 解析自定义控制器数据 - 6字节 (偏移 24-29) */
    MiniPC_Data.main_buttons   = pbuf[idx++];
    MiniPC_Data.handle_buttons = pbuf[idx++];
    /* 从 handle_buttons 位域拆出独立按键标志, 适配 MINIPC_KEY_RISING_EDGE 边沿检测宏 */
    MiniPC_Data.btn1 = (MiniPC_Data.handle_buttons >> 0) & 0x01U;
    MiniPC_Data.btn2 = (MiniPC_Data.handle_buttons >> 1) & 0x01U;
    MiniPC_Data.btn3 = (MiniPC_Data.handle_buttons >> 2) & 0x01U;
    MiniPC_Data.btn4 = (MiniPC_Data.handle_buttons >> 3) & 0x01U;
    MiniPC_Data.joystick_x     = pbuf[idx] | (pbuf[idx + 1] << 8);
    idx += 2;
    MiniPC_Data.joystick_y     = pbuf[idx] | (pbuf[idx + 1] << 8);
    idx += 2;

    /* 解析3个int16_t - 鼠标数据 (偏移 30-35) */
    MiniPC_Data.mouse_x = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
    idx += 2;
    MiniPC_Data.mouse_y = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
    idx += 2;
    MiniPC_Data.mouse_z = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
    idx += 2;

    /* 解析21个uint8_t - 鼠标按键和键盘按键 (偏移 30-50) */
    MiniPC_Data.mouse_left  = pbuf[idx++];
    MiniPC_Data.mouse_right = pbuf[idx++];
    MiniPC_Data.mouse_mid   = pbuf[idx++];
    MiniPC_Data.key_w       = pbuf[idx++];
    MiniPC_Data.key_s       = pbuf[idx++];
    MiniPC_Data.key_a       = pbuf[idx++];
    MiniPC_Data.key_d       = pbuf[idx++];
    MiniPC_Data.key_shift   = pbuf[idx++];
    MiniPC_Data.key_ctrl    = pbuf[idx++];
    MiniPC_Data.key_q       = pbuf[idx++];
    MiniPC_Data.key_e       = pbuf[idx++];
    MiniPC_Data.key_r       = pbuf[idx++];
    MiniPC_Data.key_f       = pbuf[idx++];
    MiniPC_Data.key_g       = pbuf[idx++];
    MiniPC_Data.key_z       = pbuf[idx++];
    MiniPC_Data.key_x       = pbuf[idx++];
    MiniPC_Data.key_c       = pbuf[idx++];
    MiniPC_Data.key_v       = pbuf[idx++];
    MiniPC_Data.key_b       = pbuf[idx++];
    MiniPC_Data.key_1       = pbuf[idx++];
    MiniPC_Data.key_2       = pbuf[idx++];

    /* 收到完整有效帧 → 复位在线计数, 清除离线标志 */
    MiniPC_Data.online_cnt = 0xFAU;
    MiniPC_Data.lost       = false;
}

/**
 * @brief  MiniPC 离线监测
 * @note   用法与 Remote_Message_Moniter / DM_Motor_Offline_Monitor 一致:
 *         - 每收到一帧有效 USB 数据, MiniPC_Receive_Info 将 online_cnt 重置为 0xFA
 *         - 本函数每主循环调用, 递减 online_cnt
 *         - 连续约 200 次循环未收到新数据 (online_cnt ≤ 0x32) 视为离线,
 *           清零数据并置 lost = true, 直到下一帧有效数据恢复
 */
void MiniPC_Offline_Monitor(void)
{
    if (MiniPC_Data.online_cnt <= 0x32U)
    {
        MiniPC_Data.lost = true;
    }
    else if (MiniPC_Data.online_cnt > 0)
    {
        MiniPC_Data.online_cnt--;
    }
}

void MiniPC_Data_Update_Last(void)
{
    MiniPC_Data_Last = MiniPC_Data;
}
