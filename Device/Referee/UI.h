/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.h
  * @brief          : 选手端图传画面 UI 绘制 (RoboMaster 2026 协议 V1.3.0)
  *                   通过 USART1 向裁判系统发送图形绘制指令
  * @author         : sanyue (适配: Engineer_2026_H7_Template_new)
  * @date           : 2026/04/26
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 0x0103 几何 + 多段 0x0110 + 0x0101 浮点数(UI_FLOAT)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef UI_H
#define UI_H

#include "stdint.h"
#include "stdbool.h"

/*********************************************************************************************************
*                                              ui绘制参数
*  屏幕坐标: (0,0) 为屏幕左下角, (1920,1080) 为屏幕右上角
*********************************************************************************************************/
#define SCREEN_RES_WIDTH            1920U
#define SCREEN_RES_HEIGHT           1080U

#define RECTANGLE_START_X           (SCREEN_RES_WIDTH / 2U + 740U)
#define RECTANGLE_START_Y           (SCREEN_RES_HEIGHT / 2U + 210U)
#define RECTANGLE_WIDTH              210
#define RECTANGLE_HEIGHT             120
#define RECTANGLE_LINE_WIDTH         6

/* 横线 1：矩形垂直方向正中 */
#define LINE1_START_X               0
#define LINE1_START_Y               0
#define LINE1_END_X                 0
#define LINE1_END_Y                 0
#define LINE1_WIDTH                 3

/* 横线 2：矩形靠上约 1/4 高度处 */
#define LINE2_START_X               0
#define LINE2_START_Y               0
#define LINE2_END_X                 0
#define LINE2_END_Y                 0
#define LINE2_WIDTH                 3

#define LINE3_START_X               (SCREEN_RES_WIDTH / 2U + 166)
#define LINE3_START_Y               0
#define LINE3_END_X                 (SCREEN_RES_WIDTH / 2U + 32)
#define LINE3_END_Y                 (SCREEN_RES_HEIGHT / 2U)
#define LINE3_WIDTH                 RECTANGLE_LINE_WIDTH

#define LINE4_START_X               (SCREEN_RES_WIDTH / 2U - 339)
#define LINE4_START_Y               0
#define LINE4_END_X                 (SCREEN_RES_WIDTH / 2U - 133)
#define LINE4_END_Y                 (SCREEN_RES_HEIGHT / 2U)
#define LINE4_WIDTH                 RECTANGLE_LINE_WIDTH
/* 档位文字：表 2-27，左下角锚点 */
#define TEXT_STRING_UP              "UP"
#define TEXT_STRING_MID             "MID"
#define TEXT_STRING_DOWN            "DOWN"
#define TEXT_STRING_BRACE           "BRACE"
#define TEXT_STRING_CLIMB           "CLIMB"
#define TEXT_LEN_UP                 (sizeof(TEXT_STRING_UP) - 1U)
#define TEXT_LEN_MID                (sizeof(TEXT_STRING_MID) - 1U)
#define TEXT_LEN_DOWN               (sizeof(TEXT_STRING_DOWN) - 1U)
#define TEXT_LEN_BRACE              (sizeof(TEXT_STRING_BRACE) - 1U)
#define TEXT_LEN_CLIMB              (sizeof(TEXT_STRING_CLIMB) - 1U)
#define TEXT_FONT_SIZE              28U
#define TEXT_LINE_WIDTH             4U
#define TEXT_START_X                (SCREEN_RES_WIDTH / 2U + 750U)
#define TEXT_START_Y                (SCREEN_RES_HEIGHT / 2U + 280U)

/* 第二段字符：ENABLE / DISABLE；位置与 UI.c 中 chassis_info.mode 对应关系见注释 */
#define TEXT_MODE_ENABLE             "ENABLE"
#define TEXT_MODE_DISABLE            "DISABLE"
#define TEXT_MODE_ENABLE_LEN         (sizeof(TEXT_MODE_ENABLE) - 1U)
#define TEXT_MODE_DISABLE_LEN        (sizeof(TEXT_MODE_DISABLE) - 1U)
#define TEXT_MODE_FONT_SIZE          28U
#define TEXT_MODE_LINE_WIDTH         4U
#define TEXT_MODE_START_X            (SCREEN_RES_WIDTH / 2U + 750U)
#define TEXT_MODE_START_Y            (SCREEN_RES_HEIGHT / 2U + 314U)

/* 档位文字 */
#define TEXT_GEAR_FAST               "FAST"
#define TEXT_GEAR_SLOW               "SLOW"
#define TEXT_GEAR_FAST_LEN           (sizeof(TEXT_GEAR_FAST) - 1U)
#define TEXT_GEAR_SLOW_LEN           (sizeof(TEXT_GEAR_SLOW) - 1U)
#define TEXT_GEAR_FONT_SIZE          28U
#define TEXT_GEAR_LINE_WIDTH         4U
#define TEXT_GEAR_START_X            (SCREEN_RES_WIDTH / 2U + 750U)
#define TEXT_GEAR_START_Y            (SCREEN_RES_HEIGHT / 2U + 246U)

/* 夹爪状态文字：ON / OFF，显示在档位文字下方 */
#define TEXT_GRIPPER_ON               "OPEN"
#define TEXT_GRIPPER_OFF              "CLOSE"
#define TEXT_GRIPPER_ON_LEN           (sizeof(TEXT_GRIPPER_ON) - 1U)
#define TEXT_GRIPPER_OFF_LEN          (sizeof(TEXT_GRIPPER_OFF) - 1U)
#define TEXT_GRIPPER_FONT_SIZE        28U
#define TEXT_GRIPPER_LINE_WIDTH       4U
#define TEXT_GRIPPER_START_X          (SCREEN_RES_WIDTH / 2U + 750U)
#define TEXT_GRIPPER_START_Y          (SCREEN_RES_HEIGHT / 2U + 212U)

/* 关节电机状态文字：全部在线显示 "ALLRIGHT"，否则显示最先断线电机的 J(ID) */
#define TEXT_JOINT_ALLRIGHT            "ALLRIGHT"
#define TEXT_JOINT_ALLRIGHT_LEN        (sizeof(TEXT_JOINT_ALLRIGHT) - 1U)
#define TEXT_JOINT_FONT_SIZE           28U
#define TEXT_JOINT_LINE_WIDTH          4U
#define TEXT_JOINT_START_X             (SCREEN_RES_WIDTH / 2U + 750U)
#define TEXT_JOINT_START_Y             (SCREEN_RES_HEIGHT / 2U + 178U)

/* 表 2-23 浮点数：details_a=字体大小，details_b 无作用；details_c~e 共 32 位整型，显示值=整型/1000 */
#define FLOAT_TEST_FONT_SIZE        40U
#define FLOAT_TEST_LINE_WIDTH       4U       /* 建议 字体:线宽 ≈ 10:1 */
#define FLOAT_TEST_START_X          (SCREEN_RES_WIDTH / 2U - 900U)
#define FLOAT_TEST_START_Y          (SCREEN_RES_HEIGHT / 2U + 314U)

/*********************************************************************************************************
*                                              裁判系统协议常量
*********************************************************************************************************/
#define REFEREE_HEADER_SOF          0xA5U
#define REFEREE_LEN_FRAME_HEAD      5U
#define REFEREE_CMD_ID_INTERACT     0x0301U
#define UI_INTERACT_ID_DRAW_ONE     0x0101U
#define UI_INTERACT_ID_DRAW_TWO     0x0102U
#define UI_INTERACT_ID_DRAW_FIVE    0x0103U   /* 表 2-25：5 个 15B 图形 */
#define UI_INTERACT_ID_DRAW_CHAR    0x0110U

#define UI_LEN_REFEREE_DATA_ONE     (6U + 15U)        /* 0x0101: 21 */
#define UI_LEN_REFEREE_DATA_FIVE    (6U + 15U * 5U)   /* 0x0103: 81 */
#define UI_LEN_REFEREE_DATA_CHAR    (6U + 15U + 30U)  /* 0x0110: 51 */

/*********************************************************************************************************
*                                              图形操作 / 类型 / 颜色枚举 (协议表 1-27)
*********************************************************************************************************/
typedef enum
{
    UI_NONE     = 0,
    UI_ADD      = 1,
    UI_MODIFY   = 2,
    UI_DELETE   = 3,
} Graphic_Operate_e;

typedef enum
{
    UI_LINE         = 0,
    UI_RECTANGLE    = 1,
    UI_CIRCLE       = 2,
    UI_OVAL         = 3,
    UI_ARC          = 4,
    UI_FLOAT        = 5,
    UI_INT          = 6,
    UI_CHAR         = 7,
} Graphic_Type_e;

typedef enum
{
    UI_LAYER_0 = 0, UI_LAYER_1, UI_LAYER_2, UI_LAYER_3, UI_LAYER_4,
    UI_LAYER_5,     UI_LAYER_6, UI_LAYER_7, UI_LAYER_8, UI_LAYER_9,
} Graphic_Layer_e;

typedef enum
{
    UI_SELF_COLOR   = 0,
    UI_YELLOW       = 1,
    UI_GREEN        = 2,
    UI_ORANGE       = 3,
    UI_FUCHSIA      = 4,
    UI_PINK         = 5,
    UI_CYAN_BLUE    = 6,
    UI_BLACK        = 7,
    UI_WHITE        = 8,
} Graphic_Color_e;

/*********************************************************************************************************
*                                              数据结构
*********************************************************************************************************/
#pragma pack(1)

typedef struct
{
    uint8_t  SOF;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  CRC8;
} frame_header_t;

typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} interaction_header_t;

/* 0x0101：5 + 2 + 21 + 2 = 30 */
typedef struct
{
    frame_header_t       frame_header;
    uint16_t             cmd_id;
    interaction_header_t interact_header;
    uint8_t              graphic[15];
    uint16_t             frame_tail_crc16;
} ext_graphic_one_data_t;

/* 0x0103：5 + 2 + 81 + 2 = 90 */
typedef struct
{
    frame_header_t       frame_header;
    uint16_t             cmd_id;
    interaction_header_t interact_header;
    uint8_t              graphic[75];
    uint16_t             frame_tail_crc16;
} ext_graphic_five_data_t;

/* 0x0110：5 + 2 + 51 + 2 = 60 */
typedef struct
{
    frame_header_t       frame_header;
    uint16_t             cmd_id;
    interaction_header_t interact_header;
    uint8_t              char_config[15];
    uint8_t              char_data[30];
    uint16_t             frame_tail_crc16;
} ext_graphic_char_data_t;

#pragma pack()

extern void UI_Tick(void);

#endif //UI_H
