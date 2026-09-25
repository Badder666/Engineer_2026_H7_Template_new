/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo.h
  * @brief          : 总线舵机驱动 (UART) - JOHO 协议 v1.1
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.1
  * @note           : 适配 JOHO UART 总线舵机通信协议 (2022.05.25)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DEVICE_SERVO_H
#define DEVICE_SERVO_H

#include "stdint.h"
#include "stdbool.h"

/* 前向声明: Servo_RxEventCallback 使用 UART_HandleTypeDef* (仅指针, 无需完整定义) */
#ifndef UART_HandleTypeDef
struct __UART_HandleTypeDef;
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 位置范围 ======================== */
/* 位置 0~4095 对应 -180°~180° */
#define SERVO_POS_MIN       0
#define SERVO_POS_MAX       4095
#define SERVO_POS_MID       2048

/* ======================== ID 范围 ======================== */
#define SERVO_ID_MIN        1
#define SERVO_ID_MAX        250
#define SERVO_ID_BROADCAST  254

/* ======================== 指令类型 ======================== */
#define SERVO_CMD_PING          0x01    /* 查询舵机状态 */
#define SERVO_CMD_READ          0x02    /* 读数据 */
#define SERVO_CMD_WRITE         0x03    /* 写数据 */
#define SERVO_CMD_REG_WRITE     0x04    /* 异步写 (配合 ACTION 指令执行) */
#define SERVO_CMD_ACTION        0x05    /* 执行异步写 */
#define SERVO_CMD_RESET         0x06    /* 恢复出厂设置 */
#define SERVO_CMD_SYNC_WRITE    0x83    /* 同步写多个舵机 */

/* ======================== 寄存器地址 ======================== */
#define SERVO_ADDR_ID           0x05    /* 舵机 ID (R/W, Uint8) */
#define SERVO_ADDR_STALL_TIME   0x06    /* 堵转保护时间 (R/W, Uint8) */
#define SERVO_ADDR_MIN_ANGLE    0x09    /* 最小角度限制 (R/W, Uint16) */
#define SERVO_ADDR_MAX_ANGLE    0x0B    /* 最大角度限制 (R/W, Uint16) */
#define SERVO_ADDR_TEMP_LIMIT   0x0D    /* 温度保护阀值 (R/W, Uint8) */
#define SERVO_ADDR_MAX_TORQUE   0x10    /* 最大扭矩 (R/W, Uint16) */
#define SERVO_ADDR_MID_ADJUST   0x14    /* 中位调整 (R/W, Int16) */
#define SERVO_ADDR_MODE         0x1C    /* 舵机/电机模式 (R/W, Uint8) */
#define SERVO_ADDR_MOTOR_DIR    0x1D    /* 电机模式方向 (R/W, Uint8) */
#define SERVO_ADDR_BAUDRATE     0x1E    /* 波特率 (R/W, Uint8) */
#define SERVO_ADDR_TORQUE       0x28    /* 扭矩开关 (R/W, Uint8) */
#define SERVO_ADDR_TARGET_POS   0x2A    /* 目标位置 (R/W, Uint16) */
#define SERVO_ADDR_RUN_TIME     0x2C    /* 运行时间 (R/W, Uint16) */
#define SERVO_ADDR_CURRENT_POS  0x38    /* 当前位置 (R, Uint16) */
#define SERVO_ADDR_SAVED_POS    0x3C    /* 运行保存的目标位置 (W, Uint8) */
#define SERVO_ADDR_REG_FLAG     0x40    /* REG WRITE 标志 (R, Uint8) */
#define SERVO_ADDR_SPEED_ADJ    0x41    /* 速度调整 (R/W, Int16) */

/* ======================== 应答状态位 ======================== */
#define SERVO_STATUS_STALL       (1 << 4)  /* 堵转异常保护中 */
#define SERVO_STATUS_OVERCURRENT (1 << 3)  /* 过流异常保护中 */
#define SERVO_STATUS_OVERTEMP    (1 << 2)  /* 过温异常保护中 */
#define SERVO_STATUS_OVERVOLT    (1 << 1)  /* 过压异常保护中 */
#define SERVO_STATUS_UNDERVOLT   (1 << 0)  /* 欠压异常保护中 */

/* ======================== 响应数据结构 ======================== */
typedef struct {
    uint8_t id;             /* 应答舵机 ID */
    uint8_t status;         /* 舵机状态字节 (见 SERVO_STATUS_xxx) */
    uint8_t param_len;      /* 参数长度 */
    uint8_t params[8];      /* 参数数据 (大端) */
} Servo_ResponsePacket;

/**
 * @brief  舵机应答回调函数类型
 * @param  resp  解析后的应答数据包 (只在本回调调用期间有效)
 */
typedef void (*Servo_ResponseCallback)(const Servo_ResponsePacket *resp);

/* ======================== 控制 API ======================== */

/**
 * @brief  设置舵机目标位置
 * @param  id       舵机 ID (1~250, 可用广播 ID 254)
 * @param  position 目标位置 (0~4095)
 * @param  run_time 运行时间 (ms), 0=最快速度
 */
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t run_time);

/**
 * @brief  使能/关闭舵机扭矩输出
 * @param  id     舵机 ID
 * @param  enable true=扭矩开, false=扭矩关
 */
void Servo_SetTorque(uint8_t id, bool enable);

/**
 * @brief  设置舵机模式
 * @param  id   舵机 ID
 * @param  mode 0x01=舵机模式, 0x00=电机模式
 */
void Servo_SetMode(uint8_t id, uint8_t mode);

/**
 * @brief  读取当前位置 (异步, 结果通过 Servo_ResponseCallback 返回)
 * @param  id 舵机 ID (不可使用广播 ID)
 */
void Servo_ReadPosition(uint8_t id);

/**
 * @brief  查询舵机状态 (PING)
 * @param  id 舵机 ID (不可使用广播 ID)
 */
void Servo_Ping(uint8_t id);

/**
 * @brief  恢复出厂设置
 * @param  id 舵机 ID
 */
void Servo_Reset(uint8_t id);

/**
 * @brief  异步写目标位置 (需配合 Servo_Action 触发执行)
 * @param  id       舵机 ID (不可使用广播 ID, 仅支持地址 0x2A)
 * @param  position 目标位置 (0~4095)
 * @param  run_time 运行时间 (ms)
 */
void Servo_RegWritePosition(uint8_t id, uint16_t position, uint16_t run_time);

/**
 * @brief  执行异步写 (触发所有已接收 REG WRITE 指令的舵机同时动作)
 *         使用广播 ID 发送, 无应答
 */
void Servo_Action(void);

/**
 * @brief  写舵机寄存器 (通用, 掉电保存取决于寄存器)
 * @param  id    舵机 ID
 * @param  addr  寄存器地址
 * @param  data  数据指针 (大端)
 * @param  len   数据长度 (字节)
 */
void Servo_WriteRegister(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len);

/* ======================== 状态与回调 ======================== */

/**
 * @brief  查询舵机驱动是否正在 DMA 发送
 * @retval true=忙, false=空闲
 */
bool Servo_IsBusy(void);

/**
 * @brief  注册舵机应答回调
 * @param  callback 回调函数指针 (设为 NULL 可取消注册)
 * @note   当收到有效的舵机应答包时, 驱动会解析并调用此回调
 */
void Servo_SetResponseCallback(Servo_ResponseCallback callback);

/**
 * @brief  DMA 发送完成回调 (由 bsp_uart.c 的 HAL_UART_TxCpltCallback 调用)
 * @note   if (huart == &huart7) Servo_TxCpltCallback();
 */
void Servo_TxCpltCallback(void);

/**
 * @brief  UART RX 事件回调 (由 bsp_uart.c 的 HAL_UARTEx_RxEventCallback 调用)
 * @param  huart UART 句柄
 * @param  size  接收到的字节数
 * @note   if (huart == &huart7) Servo_RxEventCallback(huart, size);
 */
void Servo_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

/* ======================== 调试信息 ======================== */

/**
 * @brief  获取累计发送帧数
 */
uint32_t Servo_GetTxCount(void);

/**
 * @brief  获取 DMA 发送完成次数
 */
uint32_t Servo_GetTxOkCount(void);

/**
 * @brief  获取收到有效应答次数
 */
uint32_t Servo_GetRxCount(void);

/**
 * @brief  获取最后发送的帧原始字节
 * @param  len 输出参数, 接收帧长度
 * @retval 指向帧数据的指针
 */
const uint8_t *Servo_GetLastFrame(uint8_t *len);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_SERVO_H */
