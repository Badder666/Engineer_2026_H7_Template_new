/**
  ******************************************************************************
  * @file    lcd.c
  * @brief   LCD driver for ST7789 / similar (SPI1, 280×240)
  *
  * 硬件初始化 (自包含, 在 LCD_Init 中完成):
  *   - 释放 PB3: 切换调试端口到 SWD-only (__HAL_SYSCFG_REMAP_SWJ)
  *   - SPI1: PD7(MOSI), PB3(SCK) — 时钟源复用 SPI2 已配置的 PLL3
  *   - GPIO: PE15(CS), PB10(BLK), PB11(RES), PD10(DC)
  *
  * 移植自 CtrBoard-H7_LCD
  ******************************************************************************
  */

#include "lcd.h"
#include "lcdfont.h"

/* SPI1 句柄 — 自包含, 不依赖 CubeMX 生成的 spi.c */
static SPI_HandleTypeDef hspi1;

/* ===================== 硬件初始化 (私有) ===================== */

/**
  * @brief  初始化 LCD 控制 GPIO (CS, DC, RES, BLK)
  */
static void lcd_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PE15 — CS */
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* PB10 — BLK, PB11 — RES */
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PD10 — DC */
    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* 初始电平 */
    LCD_CS_Set();
    LCD_BLK_Clr();
    LCD_RES_Set();
    LCD_DC_Set();
}

/**
  * @brief  初始化 SPI1 (PD7=MOSI, PB3=SCK)
  * @note   SPI123 时钟源由 SPI2 的 MSP Init 已设为 PLL3, 此处不再重复配置。
  *         仅使能 SPI1 时钟 + 配置 GPIO + HAL 初始化。
  */
static void lcd_spi_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* SPI1 时钟使能 */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* PD7 — SPI1_MOSI (AF5) */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* PB3 — SPI1_SCK (AF5) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* SPI1 参数: 主机, TX-only, 8bit, CPOL=1 CPHA=1, 预分频 /8 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 0x0;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/* ===================== SPI 写字节 ===================== */

void LCD_Writ_Bus(uint8_t dat)
{
    LCD_CS_Clr();
#if USE_ANALOG_SPI
    for (uint8_t i = 0; i < 8; i++) {
        LCD_SCLK_Clr();
        if (dat & 0x80) LCD_MOSI_Set();
        else            LCD_MOSI_Clr();
        LCD_SCLK_Set();
        dat <<= 1;
    }
#else
    HAL_SPI_Transmit(&hspi1, &dat, 1, 0xffff);
#endif
    LCD_CS_Set();
}

/* ===================== LCD 协议函数 ===================== */

void LCD_WR_DATA8(uint8_t dat)  { LCD_Writ_Bus(dat); }

void LCD_WR_DATA(uint16_t dat)
{
    LCD_Writ_Bus(dat >> 8);
    LCD_Writ_Bus(dat);
}

void LCD_WR_REG(uint8_t dat)
{
    LCD_DC_Clr();
    LCD_Writ_Bus(dat);
    LCD_DC_Set();
}

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (USE_HORIZONTAL == 0) {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1); LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1 + 20); LCD_WR_DATA(y2 + 20);
        LCD_WR_REG(0x2c);
    } else if (USE_HORIZONTAL == 1) {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1); LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1 + 20); LCD_WR_DATA(y2 + 20);
        LCD_WR_REG(0x2c);
    } else if (USE_HORIZONTAL == 2) {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1 + 20); LCD_WR_DATA(x2 + 20);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1); LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);
    } else {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1 + 20); LCD_WR_DATA(x2 + 20);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1); LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);
    }
}

/* ===================== 绘图 API ===================== */

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t i, j;
    LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);
    for (i = ysta; i < yend; i++) {
        for (j = xsta; j < xend; j++) {
            LCD_WR_DATA(color);
        }
    }
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0) incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }

    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }

    if (delta_x > delta_y) distance = delta_x;
    else distance = delta_y;

    for (t = 0; t < distance + 1; t++) {
        LCD_DrawPoint(uRow, uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) { xerr -= distance; uRow += incx; }
        if (yerr > distance) { yerr -= distance; uCol += incy; }
    }
}

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a = 0, b = r;
    while (a <= b) {
        LCD_DrawPoint(x0 - b, y0 - a, color);
        LCD_DrawPoint(x0 + b, y0 - a, color);
        LCD_DrawPoint(x0 - a, y0 + b, color);
        LCD_DrawPoint(x0 - a, y0 - b, color);
        LCD_DrawPoint(x0 + b, y0 + a, color);
        LCD_DrawPoint(x0 + a, y0 - b, color);
        LCD_DrawPoint(x0 + a, y0 + b, color);
        LCD_DrawPoint(x0 - b, y0 + a, color);
        a++;
        if ((a * a + b * b) > (r * r)) b--;
    }
}

/* ===================== 中文显示 ===================== */

void LCD_ShowChinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    while (*s != 0) {
        if (sizey == 12)      LCD_ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 16) LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 24) LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 32) LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
        else return;
        s += 2;
        x += sizey;
    }
}

void LCD_ShowChinese12x12(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t x0 = x;

    HZnum = sizeof(tfont12) / sizeof(typFNT_GB12);
    for (k = 0; k < HZnum; k++) {
        if ((tfont12[k].Index[0] == *(s)) && (tfont12[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont12[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) { m = 0; break; }
                    } else {
                        if (tfont12[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) { x = x0; y++; break; }
                    }
                }
            }
        }
        continue;
    }
}

void LCD_ShowChinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t x0 = x;

    HZnum = sizeof(tfont16) / sizeof(typFNT_GB16);
    for (k = 0; k < HZnum; k++) {
        if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont16[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) { m = 0; break; }
                    } else {
                        if (tfont16[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) { x = x0; y++; break; }
                    }
                }
            }
        }
        continue;
    }
}

void LCD_ShowChinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t x0 = x;

    HZnum = sizeof(tfont24) / sizeof(typFNT_GB24);
    for (k = 0; k < HZnum; k++) {
        if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont24[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) { m = 0; break; }
                    } else {
                        if (tfont24[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) { x = x0; y++; break; }
                    }
                }
            }
        }
        continue;
    }
}

void LCD_ShowChinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;
    uint16_t TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t x0 = x;

    HZnum = sizeof(tfont32) / sizeof(typFNT_GB32);
    for (k = 0; k < HZnum; k++) {
        if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) {
                        if (tfont32[k].Msk[i] & (0x01 << j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if (m % sizey == 0) { m = 0; break; }
                    } else {
                        if (tfont32[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if ((x - x0) == sizey) { x = x0; y++; break; }
                    }
                }
            }
        }
        continue;
    }
}

/* ===================== ASCII 字符 ===================== */

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum;
    uint16_t x0 = x;

    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';

    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);
    for (i = 0; i < TypefaceNum; i++) {
        if (sizey == 12)      temp = ascii_1206[num][i];
        else if (sizey == 16) temp = ascii_1608[num][i];
        else if (sizey == 24) temp = ascii_2412[num][i];
        else if (sizey == 32) temp = ascii_3216[num][i];
        else return;

        for (t = 0; t < 8; t++) {
            if (!mode) {
                if (temp & (0x01 << t)) LCD_WR_DATA(fc);
                else LCD_WR_DATA(bc);
                m++;
                if (m % sizex == 0) { m = 0; break; }
            } else {
                if (temp & (0x01 << t)) LCD_DrawPoint(x, y, fc);
                x++;
                if ((x - x0) == sizex) { x = x0; y++; break; }
            }
        }
    }
}

void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    while (*p != '\0') {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}

/* ===================== 数字 ===================== */

uint32_t mypow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t sizex = sizey / 2;

    for (t = 0; t < len; t++) {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
                continue;
            } else enshow = 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    int32_t num_int;
    uint8_t t, temp, sizex;
    uint8_t pos_x;
    uint8_t is_negative;
    sizex = sizey / 2;

    is_negative = (num < 0);
    if (is_negative) num = -num;
    num_int = (int32_t)(num * mypow(10, decimal) + 0.5f);

    /* 符号位 */
    if (is_negative) {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
    } else {
        LCD_ShowChar(x, y, ' ', fc, bc, sizey, 0);
    }

    pos_x = x + sizex;

    /* 清除背景: 整数(len) + 小数点(1) + 小数(decimal) */
    LCD_Fill(pos_x, y, pos_x + (len + 1 + decimal) * sizex, y + sizey + 1, bc);

    /* 依次显示所有数字: 先整数位, 再小数点, 再小数位 */
    for (t = 0; t < len + decimal; t++) {
        if (t == len) {
            LCD_ShowChar(pos_x, y, '.', fc, bc, sizey, 0);
            pos_x += sizex;
        }
        temp = ((num_int / mypow(10, len + decimal - t - 1)) % 10) + '0';
        LCD_ShowChar(pos_x, y, temp, fc, bc, sizey, 0);
        pos_x += sizex;
    }
}

void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    int32_t num_int;
    uint8_t t, temp, sizex;
    uint8_t pos_x;
    sizex = sizey / 2;

    /* 取绝对值, 四舍五入 */
    if (num < 0) num = -num;
    num_int = (int32_t)(num * mypow(10, decimal) + 0.5f);

    pos_x = x + sizex;

    /* 清除背景: 整数(len) + 小数点(1) + 小数(decimal) */
    LCD_Fill(pos_x, y, pos_x + (len + 1 + decimal) * sizex, y + sizey + 1, bc);

    /* 依次显示所有数字 */
    for (t = 0; t < len + decimal; t++) {
        if (t == len) {
            LCD_ShowChar(pos_x, y, '.', fc, bc, sizey, 0);
            pos_x += sizex;
        }
        temp = ((num_int / mypow(10, len + decimal - t - 1)) % 10) + '0';
        LCD_ShowChar(pos_x, y, temp, fc, bc, sizey, 0);
        pos_x += sizex;
    }
}

/* ===================== 图片 ===================== */

void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
    uint16_t i, j;
    uint32_t k = 0;
    LCD_Address_Set(x, y, x + length - 1, y + width - 1);
    for (i = 0; i < length; i++) {
        for (j = 0; j < width; j++) {
            LCD_WR_DATA8(pic[k * 2]);
            LCD_WR_DATA8(pic[k * 2 + 1]);
            k++;
        }
    }
}

/* ===================== LCD 初始化 ===================== */

void LCD_Init(void)
{
    /* ========== 阶段 0: 上电稳定 (等待 LCD 内部电荷泵电压建立) ========== */
    HAL_Delay(200);

    /* ---- 释放 PB3: 切换调试端口到 SWD-only 模式 ---- */
    /* PB3 复位后默认为 JTDO, 需要释放以作 SPI1_SCK 使用 */
    /* SW_CFG[1:0] = 10: SW-DP only, JTAG-DP disabled */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    SYSCFG->CCCSR = (SYSCFG->CCCSR & ~0xC0000000U) | 0x80000000U;

    /* ---- GPIO 初始化 ---- */
    lcd_gpio_init();

    /* ========== 阶段 1: 提前复位 (在 SPI 初始化前拉低 RESET) ========== */
    /* 确保 SPI 引脚配置时的毛刺被 LCD 忽略 */
    LCD_RES_Clr();
    HAL_Delay(150);

    /* ========== 阶段 2: SPI1 初始化 ========== */
    lcd_spi_init();

    /* SPI 空操作: 刷掉 FIFO 中可能存在的残留数据 */
    {
        uint8_t dummy = 0x00;
        HAL_SPI_Transmit(&hspi1, &dummy, 1, 0xffff);
    }

    /* ========== 阶段 3: 释放复位 ========== */
    LCD_RES_Set();
    HAL_Delay(150);

    /* ========== 阶段 4: LCD 驱动 IC 初始化序列 (ST7789) ========== */
    LCD_WR_REG(0x11);   /* Sleep out */
    HAL_Delay(150);

    /* Memory Data Access Control */
    LCD_WR_REG(0x36);
    if (USE_HORIZONTAL == 0)      LCD_WR_DATA8(0x00);
    else if (USE_HORIZONTAL == 1) LCD_WR_DATA8(0xC0);
    else if (USE_HORIZONTAL == 2) LCD_WR_DATA8(0x70);
    else                          LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);   /* 16-bit color (65K) */

    /* Porch settings */
    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);

    LCD_WR_REG(0xB7);
    LCD_WR_DATA8(0x35);

    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x32);   /* Vcom = 1.35V */

    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x15);   /* GVDD = 4.8V */

    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20);   /* VDV, 0x20:0v */

    LCD_WR_REG(0xC6);
    LCD_WR_DATA8(0x0F);   /* 60Hz */

    LCD_WR_REG(0xD0);
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);

    /* Positive gamma */
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xD0); LCD_WR_DATA8(0x08); LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x33); LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17); LCD_WR_DATA8(0x14); LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x34);

    /* Negative gamma */
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xD0); LCD_WR_DATA8(0x08); LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x33); LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17); LCD_WR_DATA8(0x14); LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x34);

    LCD_WR_REG(0x21);   /* Display inversion on */
    LCD_WR_REG(0x29);   /* Display on */

    /* ========== 阶段 5: 背光 (初始化完成后再开启, 避免电源毛刺) ========== */
    LCD_BLK_Set();
    HAL_Delay(50);
}
