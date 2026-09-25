/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : WS2812_Task.c
  * @brief          : WS2812 LED task
  *                   全部在线 → 绿灯常亮
  *                   电机离线 → 锁定第一个离线电机, 以其代表色呼吸
  * @author         : Ported from CtrBoard-H7_WS2812
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "WS2812.h"
#include "Motor_DM.h"       /* Robotic_Arm_Motor[].Data.offline */

/* ======================== 状态机 ======================== */
typedef enum {
    WS_STATE_INIT,          /* 上电初始化 — 灯灭, 等待电机上线         */
    WS_STATE_ALL_ONLINE,    /* 全部在线 — 绿灯常亮                    */
    WS_STATE_OFFLINE,       /* 有电机离线 — 锁定第一个离线的颜色呼吸   */
} WS_State_t;

/**
  * @brief  根据电机索引获取呼吸颜色 (三角波亮度)
  * @param  idx    电机索引 0~5
  * @param  bright 当前呼吸亮度 0~255
  * @param  r/g/b  输出颜色
  * @note   6 种颜色尽量分布在色环上, 肉眼易区分
  */
static void WS2812_GetMotorColor(int8_t idx, uint8_t bright,
                                  uint8_t *r, uint8_t *g, uint8_t *b)
{
    static const struct {
        uint8_t fr, fg, fb;  /* 各通道占 bright 的份数 (0~255) */
    } kColorMap[6] = {
        /* 电机 0 ~ 5 的分配比例 */
        { 255,   0,   0 },   /* 0 — 红 Red        */
        { 255,  64,   0 },   /* 1 — 橙 Orange     */
        { 255, 255,   0 },   /* 2 — 黄 Yellow     */
        {   0, 255,   0 },   /* 3 — 绿 Green      */
        {   0,   0, 255 },   /* 4 — 蓝 Blue       */
        { 128,   0, 255 },   /* 5 — 紫 Purple     */
    };

    if (idx < 0 || idx >= 6)
        idx = 0;  /* 防护 */

    uint16_t r16 = (uint16_t)kColorMap[idx].fr * bright / 255;
    uint16_t g16 = (uint16_t)kColorMap[idx].fg * bright / 255;
    uint16_t b16 = (uint16_t)kColorMap[idx].fb * bright / 255;

    *r = (uint8_t)r16;
    *g = (uint8_t)g16;
    *b = (uint8_t)b16;
}

/**
  * @brief  WS2812 task: 监测机械臂 DM 电机在线状态
  *         全部在线 → 绿灯常亮
  *         任一离线 → 以该电机的颜色呼吸
  *         (锁定首次离线电机, CAN 瘫痪后也不会丢失)
  * @param  argument: Not used
  * @retval None
  */
void WS2812_Task(void const * argument)
{
    (void)argument;
    uint16_t step = 0;
    uint16_t i;

    WS_State_t state   = WS_STATE_INIT;
    int8_t     first_bad_idx = -1;   /* 锁定的首个离线电机索引, -1 = 无 */

    WS2812_Init();

    for (;;)
    {
        /* ---- 扫描当前离线情况 ---- */
        bool any_offline = false;
        int8_t first_idx = -1;

        for (i = 0; i < 6; i++)
        {
            if (Robotic_Arm_Motor[i].Data.offline)
            {
                any_offline = true;
                first_idx = (int8_t)i;
                break;
            }
        }

        /* ---- 状态机转移 ---- */
        switch (state)
        {
        case WS_STATE_INIT:
            if (!any_offline)
                state = WS_STATE_ALL_ONLINE;
            break;

        case WS_STATE_ALL_ONLINE:
            if (any_offline)
            {
                first_bad_idx = first_idx;   /* 锁定第一次离线 */
                state = WS_STATE_OFFLINE;
            }
            break;

        case WS_STATE_OFFLINE:
            if (!any_offline)
            {
                first_bad_idx = -1;
                state = WS_STATE_ALL_ONLINE; /* 全部恢复 */
            }
            break;
        }

        /* ---- 设置灯色 ---- */
        uint8_t r, g, b;

        if (state == WS_STATE_OFFLINE)
        {
            /* 三角波呼吸: 0 → 255 → 0 */
            uint8_t bright = (step < 128)
                                ? (uint8_t)(step * 2)
                                : (uint8_t)((255 - step) * 2);

            WS2812_GetMotorColor(first_bad_idx, bright, &r, &g, &b);

            step += 8;
            if (step >= 256) step = 0;
        }
        else
        {
            /* INIT: 全灭  /  ALL_ONLINE: 绿灯常亮 */
            r = 0;
            g = (state == WS_STATE_ALL_ONLINE) ? 255 : 0;
            b = 0;
            step = 0;   /* 复位呼吸相位 */
        }

        for (i = 0; i < WS2812_LED_NUM; i++)
            WS2812_SetLED(i, r, g, b);
        WS2812_Update();
        osDelay(15);
    }
}
