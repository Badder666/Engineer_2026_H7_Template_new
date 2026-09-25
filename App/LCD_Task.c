/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LCD_Task.c
  * @brief          : LCD display — servo position only
  * @note           : Shows current servo position in large text.
  *                   Servo control is in Servo_Task.c (sine wave).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "lcd.h"
#include "pic.h"
/* ========================= 任务入口 ========================= */

void LCD_Task(void const *argument)
{
    (void)argument;
    //(280×240 屏幕)
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    for (;;)
    {
        //LCD_ShowChinese(80, 30, (const uint8_t *)"\xB4\xEF\xC3\xEE", CYAN, BLACK, 24, 0);
        //LCD_ShowFloatNum(80, 30, 3.145f, 1, 3, CYAN, BLACK, 24);
        //LCD_ShowPicture(100, 82, 80, 76, gImage_1);
        osDelay(50);
    }
}
