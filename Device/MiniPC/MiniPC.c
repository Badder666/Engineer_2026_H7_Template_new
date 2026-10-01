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
#include "custom_controller_protocol.h"
#include "usb_comm.h"
#include "main.h"
#include <math.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

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
static ControllerStreamParser command_parser;

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
    if (Buf == NULL || Len != 6U) {
        return USBD_FAIL;
    }
    /* USB teardown must not free the CDC handle while a task submits a frame. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED ||
        hUsbDeviceHS.pClassData == NULL) {
        __set_PRIMASK(primask);
        return USBD_FAIL;
    }
    if (((USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData)->TxState != 0U) {
        __set_PRIMASK(primask);
        return USBD_BUSY;
    }
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
    const uint8_t result = CDC_Transmit_HS(joint_data_transmit, Len * 4 + 3);
    __set_PRIMASK(primask);
    return result;
}

static void receive_frame(const uint8_t *frame, size_t length, void *context)
{
    (void)context;
    MiniPC_DataTypeDef next = MiniPC_Data;
    if (length == CONTROLLER_COMMAND_SIZE) {
        ControllerCommand command;
        if (!ControllerCommand_Decode(frame, length, &command)) {
            return;
        }
        for (size_t i = 0; i < 6U; ++i) {
            if ((command.valid_joint_mask & (1U << i)) != 0U) {
                next.joint_pos_data[i] = command.positions[i];
            }
        }
        next.controller_buttons = command.buttons;
        next.valid_joint_mask = command.valid_joint_mask;
        next.command_sequence = command.sequence;
        next.controller_time_ms = command.timestamp_ms;
        const bool operator_valid = (command.flags & CONTROLLER_OPERATOR_VALID) != 0U;
        next.joystick_x = operator_valid ? command.joystick_x : 2048U;
        next.joystick_y = operator_valid ? command.joystick_y : 2048U;
        next.mouse_x = operator_valid ? command.mouse[0] : 0;
        next.mouse_y = operator_valid ? command.mouse[1] : 0;
        next.mouse_z = operator_valid ? command.mouse[2] : 0;
        next.mouse_left = operator_valid ? command.mouse_buttons[0] : 0U;
        next.mouse_right = operator_valid ? command.mouse_buttons[1] : 0U;
        next.mouse_mid = operator_valid ? command.mouse_buttons[2] : 0U;
        next.main_buttons = operator_valid ? command.main_buttons : 0U;
        next.handle_buttons = operator_valid ? command.handle_buttons : 0U;
        const uint32_t keys = operator_valid ? command.keys : 0U;
        uint8_t *key_fields[] = {
            &next.key_w, &next.key_s, &next.key_a, &next.key_d,
            &next.key_shift, &next.key_ctrl, &next.key_q, &next.key_e,
            &next.key_r, &next.key_f, &next.key_g, &next.key_z,
            &next.key_x, &next.key_c, &next.key_v, &next.key_b,
            &next.key_1, &next.key_2,
        };
        for (size_t i = 0; i < 18U; ++i) {
            *key_fields[i] = (uint8_t)((keys >> i) & 1U);
        }
    } else {
        /* 旧 60 字节帧已由字节流解析器校验，此处只解析有效载荷。 */
        const uint8_t *pbuf = &frame[1];
        uint32_t idx = 0;
        for (uint32_t i = 0; i < 6; i++) {
            const float position = bytes_to_float_union(&pbuf[idx]);
            if (!isfinite(position)) {
                return;
            }
            next.joint_pos_data[i] = position;
            idx += 4;
        }
        next.main_buttons   = pbuf[idx++];
        next.handle_buttons = pbuf[idx++];
        next.joystick_x     = pbuf[idx] | (pbuf[idx + 1] << 8);
        idx += 2;
        next.joystick_y     = pbuf[idx] | (pbuf[idx + 1] << 8);
        idx += 2;
        next.mouse_x = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
        idx += 2;
        next.mouse_y = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
        idx += 2;
        next.mouse_z = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
        idx += 2;
        next.mouse_left  = pbuf[idx++];
        next.mouse_right = pbuf[idx++];
        next.mouse_mid   = pbuf[idx++];
        next.key_w       = pbuf[idx++];
        next.key_s       = pbuf[idx++];
        next.key_a       = pbuf[idx++];
        next.key_d       = pbuf[idx++];
        next.key_shift   = pbuf[idx++];
        next.key_ctrl    = pbuf[idx++];
        next.key_q       = pbuf[idx++];
        next.key_e       = pbuf[idx++];
        next.key_r       = pbuf[idx++];
        next.key_f       = pbuf[idx++];
        next.key_g       = pbuf[idx++];
        next.key_z       = pbuf[idx++];
        next.key_x       = pbuf[idx++];
        next.key_c       = pbuf[idx++];
        next.key_v       = pbuf[idx++];
        next.key_b       = pbuf[idx++];
        next.key_1       = pbuf[idx++];
        next.key_2       = pbuf[idx++];
        next.controller_buttons = 0U;
        next.valid_joint_mask = 0x3FU;
        next.command_sequence = 0U;
        next.controller_time_ms = 0U;
    }
    next.btn1 = (next.handle_buttons >> 0) & 0x01U;
    next.btn2 = (next.handle_buttons >> 1) & 0x01U;
    next.btn3 = (next.handle_buttons >> 2) & 0x01U;
    next.btn4 = (next.handle_buttons >> 3) & 0x01U;

    /* 收到完整有效帧 → 复位在线计数, 清除离线标志 */
    next.online_cnt = 0xFAU;
    next.lost = false;
    MiniPC_Data = next;
}

void MiniPC_Receive_Info(void)
{
    uint8_t chunk[64];
    /* Bound work per Detect_Task iteration even if USB traffic is continuous. */
    for (size_t i = 0; i < 16U; ++i) {
        bool discontinuity;
        size_t length = USBComm_Read(chunk, sizeof(chunk), &discontinuity);
        if (discontinuity) {
            command_parser.used = 0U;
        }
        if (length == 0U) {
            break;
        }
        ControllerStream_Feed(&command_parser, chunk, length, receive_frame, NULL);
    }
}

/**
 * @brief  MiniPC 离线监测
 * @note   用法与 Remote_Message_Moniter / DM_Motor_Offline_Monitor 一致:
 *         - 每收到一帧有效 USB 数据, MiniPC_Receive_Info 将 online_cnt 重置为 0xFA
 *         - 本函数每主循环调用, 递减 online_cnt
 *         - 连续约 200 次循环未收到新数据 (online_cnt ≤ 0x32) 视为离线,
 *           置 lost = true, 保留关节目标, 直到下一帧有效数据恢复
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
