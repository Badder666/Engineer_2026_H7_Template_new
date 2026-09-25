/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.c
  * @brief          : 选手端图传画面 UI 绘制 (RoboMaster 2026 协议 V1.3.0)
  *                   矩形 + 两横线 (0x0103) + 两段 0x0110 字符（档位 + ENABLE/DISABLE）
  * @author         : sanyue (适配: Engineer_2026_H7_Template_new)
  * @date           : 2026/04/26
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 0x0103 五图形；多段 0x0110；浮点测试 0x0101(UI_FLOAT)
  ******************************************************************************
  */
/* USER CODE END Header */

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "UI.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "stm32h7xx_hal_uart.h"
#include "CRC.h"
#include "Referee_System.h"
#include "chassis_config.h"
#include "arm_config.h"
#include "Motor_DM.h"

/*********************************************************************************************************
*                                              外部变量声明
*********************************************************************************************************/
extern Referee_System_Info_TypeDef Referee_System_Info;
extern Chassis_Info_Typedef chassis_info;
extern uint8_t hand_state;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
__attribute__((section(".AXI_SRAM"))) static uint8_t ClientTxBuffer[sizeof(ext_graphic_five_data_t)];
static uint8_t ui_seq = 0;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static uint16_t Get_Self_Client_ID(uint8_t robot_id);
static void     EncodeGraphic15(uint8_t g[15],
                                const uint8_t  name[3],
                                Graphic_Operate_e op,
                                Graphic_Type_e    type,
                                Graphic_Layer_e   layer,
                                Graphic_Color_e   color,
                                uint16_t width,
                                uint16_t start_x, uint16_t start_y,
                                uint16_t details_a,
                                uint16_t details_b,
                                uint16_t details_c,
                                uint16_t details_d, uint16_t details_e);
static void     UI_Pack_And_Send_Shapes(Graphic_Operate_e op);
static void     UI_Pack_And_Send_Char(Graphic_Operate_e op);
static void     UI_Pack_And_Send_ModeChar(Graphic_Operate_e op);
static void     UI_Pack_And_Send_GearChar(Graphic_Operate_e op);
static void     UI_Pack_And_Send_GripperChar(Graphic_Operate_e op);
static void     UI_Pack_And_Send_JointChar(Graphic_Operate_e op);
static void     UI_Pack_And_Send_FloatTest(Graphic_Operate_e op);

/*********************************************************************************************************
*                                              对外调用入口
*********************************************************************************************************/

/**
  * @brief  1ms 任务中调用；约 100ms 一发。
  *         前 5 帧 ADD 几何，再依次 ADD 各段 0x0110 字符，再 5 帧 ADD 浮点测试(0x0101)，
  *         之后按 几何 / 升降档字 / 模式字 / 档位字 / 浮点 轮流 MODIFY。
  */
void UI_Tick(void)
{
    static uint16_t tick_cnt = 0;
    static uint8_t  init_cnt = 0;
    static uint8_t  tx_sel    = 0;

    if (++tick_cnt < 100) {
        return;
    }
    tick_cnt = 0;

    if (Referee_System_Info.robot_status.robot_id == 0) {
        return;
    }

    if (init_cnt < 5) {
        UI_Pack_And_Send_Shapes(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 10) {
        UI_Pack_And_Send_Char(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 15) {
        UI_Pack_And_Send_ModeChar(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 20) {
        UI_Pack_And_Send_GearChar(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 25) {
        UI_Pack_And_Send_GripperChar(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 30) {
        UI_Pack_And_Send_JointChar(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 35) {
        UI_Pack_And_Send_FloatTest(UI_ADD);
        init_cnt++;
    } else {
        switch (tx_sel % 7U) {
        case 0U:
            UI_Pack_And_Send_Shapes(UI_MODIFY);
            break;
        case 1U:
            UI_Pack_And_Send_Char(UI_MODIFY);
            break;
        case 2U:
            UI_Pack_And_Send_ModeChar(UI_MODIFY);
            break;
        case 3U:
            UI_Pack_And_Send_GearChar(UI_MODIFY);
            break;
        case 4U:
            UI_Pack_And_Send_GripperChar(UI_MODIFY);
            break;
        case 5U:
            UI_Pack_And_Send_JointChar(UI_MODIFY);
            break;
        default:
            UI_Pack_And_Send_FloatTest(UI_MODIFY);
            break;
        }
        tx_sel++;
    }
}

/*********************************************************************************************************
*                                              ui绘制 / 发送
*********************************************************************************************************/

static uint16_t Get_Self_Client_ID(uint8_t robot_id)
{
    if (robot_id >= 1 && robot_id <= 6) {
        return (uint16_t)(0x0100 + robot_id);
    } else if (robot_id >= 101 && robot_id <= 106) {
        return (uint16_t)(0x0164 + (robot_id - 100));
    }
    return 0x0102;
}

static void EncodeGraphic15(uint8_t g[15],
                            const uint8_t  name[3],
                            Graphic_Operate_e op,
                            Graphic_Type_e    type,
                            Graphic_Layer_e   layer,
                            Graphic_Color_e   color,
                            uint16_t width,
                            uint16_t start_x, uint16_t start_y,
                            uint16_t details_a,
                            uint16_t details_b,
                            uint16_t details_c,
                            uint16_t details_d, uint16_t details_e)
{
    const uint32_t da = (uint32_t)(details_a & 0x1FFU);
    const uint32_t db = (uint32_t)(details_b & 0x1FFU);
    memcpy(g, name, 3);

    const uint32_t w0 = ((uint32_t)(op & 7U))
                        | (((uint32_t)(type & 7U)) << 3)
                        | (((uint32_t)((uint8_t)layer & 0xFU)) << 6)
                        | (((uint32_t)((uint8_t)color & 0xFU)) << 10)
                        | ((da & 0x1FFU) << 14)
                        | ((db & 0x1FFU) << 23);

    const uint32_t w1 = ((uint32_t)(width & 0x3FFU))
                        | (((uint32_t)(start_x & 0x7FFU)) << 10)
                        | (((uint32_t)(start_y & 0x7FFU)) << 21);

    const uint32_t w2 = ((uint32_t)(details_c & 0x3FFU))
                        | (((uint32_t)(details_d & 0x7FFU)) << 10)
                        | (((uint32_t)(details_e & 0x7FFU)) << 21);

    memcpy(&g[3], &w0, sizeof(w0));
    memcpy(&g[7], &w1, sizeof(w1));
    memcpy(&g[11], &w2, sizeof(w2));
}

/**
  * @brief 表 2-23：整型/浮点数在图形配置 3 中占 32 位，按 c(低10)+d(11)+e(高11) 与 EncodeGraphic15 的 w2 一致
  */
static void PackInt32ToGraphicDetails(int32_t v, uint16_t *c, uint16_t *d, uint16_t *e)
{
    uint32_t u = (uint32_t)v;
    *c = (uint16_t)(u & 0x3FFU);
    *d = (uint16_t)((u >> 10) & 0x7FFU);
    *e = (uint16_t)((u >> 21) & 0x7FFU);
}

/** 空槽位：操作类型为「空操作」，占满 0x0103 后两格 */
static void EncodeGraphic15_Nop(uint8_t g[15], const uint8_t name[3])
{
    EncodeGraphic15(g, name, UI_NONE, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                    0, 0, 0, 0, 0, 0, 0, 0);
}

static void UI_Pack_And_Send_Shapes(Graphic_Operate_e op)
{
    ext_graphic_five_data_t pkt;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_FIVE;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_FIVE;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    /* 空操作占位 ①②③ — 矩形及其内部横线已移除 */
    EncodeGraphic15_Nop(pkt.graphic,      (const uint8_t[3]){'R','C','1'});
    EncodeGraphic15_Nop(&pkt.graphic[15], (const uint8_t[3]){'L','N','1'});
    EncodeGraphic15_Nop(&pkt.graphic[30], (const uint8_t[3]){'L','N','2'});
    {    
        const uint8_t nm[3] = {'L', 'N', '3'};
        EncodeGraphic15(&pkt.graphic[45], nm, op, UI_LINE, UI_LAYER_0, UI_FUCHSIA,
                        LINE3_WIDTH,
                        LINE3_START_X, LINE3_START_Y,
                        0, 0, 0,
                        LINE3_END_X, LINE3_END_Y);
    }
    {
        const uint8_t nm[3] = {'L', 'N', '4'};
        EncodeGraphic15(&pkt.graphic[60], nm, op, UI_LINE, UI_LAYER_0, UI_FUCHSIA,
                        LINE4_WIDTH,
                        LINE4_START_X, LINE4_START_Y,
                        0, 0, 0,
                        LINE4_END_X, LINE4_END_Y);
    }

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

static void UI_Pack_And_Send_Char(Graphic_Operate_e op)
{
    ext_graphic_char_data_t pkt;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_CHAR;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_CHAR;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'E', 'G', 'R'};
        if (chassis_info.lift_mode == LIFT_STAGE_1) {
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_CYAN_BLUE,
                        TEXT_LINE_WIDTH,
                        (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                        (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_UP,
                        0, 0, 0);
        }else if (chassis_info.lift_mode == LIFT_STAGE_5) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_CYAN_BLUE,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_MID,
                            0, 0, 0);
        }else if (chassis_info.lift_mode == LIFT_STAGE_3) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_CYAN_BLUE,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_DOWN,
                            0, 0, 0);
        }else if (chassis_info.lift_mode == LIFT_STAGE_4) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_CYAN_BLUE,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_BRACE,
                            0, 0, 0);
        } else if (chassis_info.lift_mode == LIFT_STAGE_2) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_CYAN_BLUE,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_CLIMB,
                            0, 0, 0);
        } else {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_CYAN_BLUE,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_UP,
                            0, 0, 0);
        }
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    if (chassis_info.lift_mode == LIFT_STAGE_1) {
        memcpy(pkt.char_data, TEXT_STRING_UP, TEXT_LEN_UP);
    } else if (chassis_info.lift_mode == LIFT_STAGE_5) {
        memcpy(pkt.char_data, TEXT_STRING_MID, TEXT_LEN_MID);
    } else if (chassis_info.lift_mode == LIFT_STAGE_3) {
        memcpy(pkt.char_data, TEXT_STRING_DOWN, TEXT_LEN_DOWN);
    } else if (chassis_info.lift_mode == LIFT_STAGE_4) {
        memcpy(pkt.char_data, TEXT_STRING_BRACE, TEXT_LEN_BRACE);
    } else if (chassis_info.lift_mode == LIFT_STAGE_2) {
        memcpy(pkt.char_data, TEXT_STRING_CLIMB, TEXT_LEN_CLIMB);
    } else {
        memcpy(pkt.char_data, TEXT_STRING_UP, TEXT_LEN_UP);
    }

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

/** 第二段字符：TEXT_MODE_ENABLE / DISABLE（宏在 UI.h），图名 MO1 与 EGR 区分 */
static void UI_Pack_And_Send_ModeChar(Graphic_Operate_e op)
{
    ext_graphic_char_data_t pkt;
    const char *mode_str;
    uint16_t    mode_len;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    if (chassis_info.mode != CHASSIS_DISABLE) {
        mode_str = TEXT_MODE_ENABLE;
        mode_len = (uint16_t)TEXT_MODE_ENABLE_LEN;
    } else {
        mode_str = TEXT_MODE_DISABLE;
        mode_len = (uint16_t)TEXT_MODE_DISABLE_LEN;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_CHAR;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_CHAR;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'M', 'O', '1'};
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_2, UI_CYAN_BLUE,
                        TEXT_MODE_LINE_WIDTH,
                        (uint16_t)TEXT_MODE_START_X, (uint16_t)TEXT_MODE_START_Y,
                        (uint16_t)TEXT_MODE_FONT_SIZE, mode_len,
                        0, 0, 0);
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    memcpy(pkt.char_data, mode_str, mode_len);

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

static void UI_Pack_And_Send_GearChar(Graphic_Operate_e op)
{
    ext_graphic_char_data_t pkt;
    const char *gear_str;
    uint16_t    gear_len;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    if (chassis_info.gear == 1) {
        gear_str = TEXT_GEAR_FAST;
        gear_len = (uint16_t)TEXT_GEAR_FAST_LEN;
    } else {
        gear_str = TEXT_GEAR_SLOW;
        gear_len = (uint16_t)TEXT_GEAR_SLOW_LEN;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_CHAR;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_CHAR;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'G', 'E', 'R'};
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_2, UI_CYAN_BLUE,
                        TEXT_GEAR_LINE_WIDTH,
                        (uint16_t)TEXT_GEAR_START_X, (uint16_t)TEXT_GEAR_START_Y,
                        (uint16_t)TEXT_GEAR_FONT_SIZE, gear_len,
                        0, 0, 0);
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    memcpy(pkt.char_data, gear_str, gear_len);

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

/** 第四段字符：夹爪状态 ON / OFF，图名 GRP，显示在档位文字下方 */
static void UI_Pack_And_Send_GripperChar(Graphic_Operate_e op)
{
    ext_graphic_char_data_t pkt;
    const char *gripper_str;
    uint16_t    gripper_len;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    if (hand_state == HAND_OPEN) {
        gripper_str = TEXT_GRIPPER_ON;
        gripper_len = (uint16_t)TEXT_GRIPPER_ON_LEN;
    } else {
        gripper_str = TEXT_GRIPPER_OFF;
        gripper_len = (uint16_t)TEXT_GRIPPER_OFF_LEN;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_CHAR;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_CHAR;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'G', 'R', 'P'};
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_2, UI_CYAN_BLUE,
                        TEXT_GRIPPER_LINE_WIDTH,
                        (uint16_t)TEXT_GRIPPER_START_X, (uint16_t)TEXT_GRIPPER_START_Y,
                        (uint16_t)TEXT_GRIPPER_FONT_SIZE, gripper_len,
                        0, 0, 0);
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    memcpy(pkt.char_data, gripper_str, gripper_len);

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

/** 第五段字符：关节电机状态 — 全部在线显示 ALLRIGHT，否则最先断线的电机 J(ID) */
static void UI_Pack_And_Send_JointChar(Graphic_Operate_e op)
{
    ext_graphic_char_data_t pkt;
    char   buf[16];
    const char *str;
    uint16_t    len;
    int8_t  first_offline = -1;
    uint8_t i;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    /* 扫描最先离线电机 */
    for (i = 0; i < 6; i++) {
        if (Robotic_Arm_Motor[i].Data.offline) {
            first_offline = (int8_t)i;
            break;
        }
    }

    if (first_offline < 0) {
        str = TEXT_JOINT_ALLRIGHT;
        len = (uint16_t)TEXT_JOINT_ALLRIGHT_LEN;
    } else {
        /* 构建 "J1" ~ "J6" */
        (void)sprintf(buf, "J%d", first_offline + 1);
        str = buf;
        len = (uint16_t)strlen(buf);
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_CHAR;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_CHAR;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'J', 'N', 'T'};
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_2, UI_CYAN_BLUE,
                        TEXT_JOINT_LINE_WIDTH,
                        (uint16_t)TEXT_JOINT_START_X, (uint16_t)TEXT_JOINT_START_Y,
                        (uint16_t)TEXT_JOINT_FONT_SIZE, len,
                        0, 0, 0);
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    memcpy(pkt.char_data, str, len);

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

/**
  * @brief 表 2-23 浮点数：类型 UI_FLOAT(5)，details_a=字体大小，details_b 无作用；
  *        details_c~e 为 32 位整型，选手端显示 = 整型/1000（例 3140 -> 3.14）。子内容 0x0101 单图。
  */
static void UI_Pack_And_Send_FloatTest(Graphic_Operate_e op)
{
    ext_graphic_one_data_t pkt;
    uint16_t                dc, dd, de;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    float test_value = Referee_System_Info.power_heat_data.buffer_energy;
    PackInt32ToGraphicDetails((int32_t)(test_value * 1000.0f), &dc, &dd, &de);

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_ONE;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_ONE;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'F', 'L', '1'};
        EncodeGraphic15(pkt.graphic, nm, op, UI_FLOAT, UI_LAYER_3, UI_CYAN_BLUE,
                        FLOAT_TEST_LINE_WIDTH,
                        (uint16_t)FLOAT_TEST_START_X, (uint16_t)FLOAT_TEST_START_Y,
                        (uint16_t)FLOAT_TEST_FONT_SIZE, 0U,
                        dc, dd, de);
    }

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}
