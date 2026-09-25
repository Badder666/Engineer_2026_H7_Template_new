/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo.c
  * @brief          : 总线舵机驱动 (UART) - JOHO 协议 v1.1
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.1
  * @note           : 适配 JOHO UART 总线舵机通信协议 (2022.05.25)
  *
  * 协议要点:
  *   - 半双工主从问答式通信
  *   - 帧头: 0xFF 0xFF (控制器→舵机) / 0xFF 0xF5 (舵机→控制器)
  *   - 多字节数据为大端模式 (高字节在前)
  *   - 校验和: ~(ID + 数据长度 + 指令类型 + 参数...) & 0xFF
  *
  * 使用方法:
  *   1. bsp_uart.c 中 HAL_UART_TxCpltCallback 内调用 Servo_TxCpltCallback()
  *   2. bsp_uart.c 中 HAL_UARTEx_RxEventCallback 内调用 Servo_RxEventCallback()
  *   3. 调用 Servo_SetResponseCallback() 注册应答回调
  *   4. 调用各控制 API 发送指令
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Servo.h"
#include "usart.h"              /* huart7 */
#include "cmsis_os.h"           /* osDelay */

/* ======================== 协议常量 ======================== */
#define FRAME_HEADER0           0xFF
#define FRAME_HEADER1           0xFF
#define FRAME_RESP_HEADER0      0xFF
#define FRAME_RESP_HEADER1      0xF5

#define SERVO_TX_BUF_SIZE       32
#define SERVO_RX_BUF_SIZE       16

/* ======================== 本地变量 ======================== */
__attribute__((section(".AXI_SRAM"))) static uint8_t  servo_tx_buf[SERVO_TX_BUF_SIZE];
__attribute__((section(".AXI_SRAM"))) static uint8_t  servo_rx_buf[SERVO_RX_BUF_SIZE];

static volatile bool servo_busy = false;

/* 调试计数器 */
static volatile uint32_t servo_tx_count   = 0;   /* 累计发送帧数 */
static volatile uint32_t servo_tx_ok_cnt  = 0;   /* DMA 发送完成次数 */
static volatile uint32_t servo_rx_cnt     = 0;   /* 收到有效应答次数 */

/* 最后发送的帧 (用于屏幕调试) */
static uint8_t  servo_last_frame[SERVO_TX_BUF_SIZE];
static uint8_t  servo_last_frame_len = 0;

/* 用户注册的应答回调 */
static Servo_ResponseCallback user_callback = NULL;

/* ======================== 静态函数声明 ======================== */
static uint8_t SERVO_Checksum(uint8_t *data, uint8_t len);
static void    SERVO_SendFrame(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len);
static void    SERVO_ParseResponse(const uint8_t *buf, uint8_t len);

/* ======================== 校验和工具函数 ======================== */

/**
  * @brief  单字节和校验
  *         checksum = (~sum(ID + data_len + cmd + params...)) & 0xFF
  * @param  data 数据指针
  * @param  len  数据长度
  * @retval 校验和
  */
static uint8_t SERVO_Checksum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (~(uint8_t)sum) & 0xFF;
}

/* ======================== 帧发送 ======================== */

/**
  * @brief  构造并发送一帧指令
  *
  * 帧格式: FF FF ID Len Cmd Param[0..N-1] CS
  *         Len = ID(1) + Cmd(1) + N = 2 + N
  *         CS  = ~(ID + Len + Cmd + Param...) & 0xFF
  *
  * @param  id        舵机 ID (1~250, 254=广播)
  * @param  cmd       指令类型 (SERVO_CMD_xxx)
  * @param  params    参数缓冲区指针 (可为 NULL)
  * @param  param_len 参数长度 (字节)
  */
static void SERVO_SendFrame(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    /* 如果 DMA 正在发送, 等待最多 2ms (115200 下 8 字节约 0.7ms) */
    if (servo_busy) {
        uint32_t timeout = 20;
        while (servo_busy && timeout--) {
            osDelay(1);
        }
        if (servo_busy) {
            return;     /* 超时, 丢弃 */
        }
    }

    /* 协议数据长度 = ID(1) + Cmd(1) + Param(N) = 2 + N */
    uint8_t data_len = 2 + param_len;
    uint8_t idx = 0;

    servo_tx_buf[idx++] = FRAME_HEADER0;
    servo_tx_buf[idx++] = FRAME_HEADER1;
    servo_tx_buf[idx++] = id;
    servo_tx_buf[idx++] = data_len;
    servo_tx_buf[idx++] = cmd;

    for (uint8_t i = 0; i < param_len; i++) {
        servo_tx_buf[idx++] = params[i];
    }

    /**
      * 校验和: ~(ID + data_len + cmd + 全部参数) & 0xFF
      * 覆盖从 servo_tx_buf[2] 开始的 3 + param_len 字节:
      *   [ID(1) + data_len(1) + cmd(1) + param(N)]
      */
    servo_tx_buf[idx] = SERVO_Checksum(&servo_tx_buf[2], 3 + param_len);

    servo_busy = true;

    /* 调试: 保存最后发送的帧 */
    servo_tx_count++;
    servo_last_frame_len = idx + 1;
    for (uint8_t i = 0; i < servo_last_frame_len; i++) {
        servo_last_frame[i] = servo_tx_buf[i];
    }

    /**
      * 对非广播指令, 提前启动接收舵机应答
      * UART7 的 TX(PE8) 和 RX(PE7) 是独立引脚, 可全双工工作,
      * 舵机在收到完整指令后才会回复, 不会与发送冲突。
      */
    if (id != SERVO_ID_BROADCAST) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart7, servo_rx_buf, SERVO_RX_BUF_SIZE);
    }

    HAL_UART_Transmit_DMA(&huart7, servo_tx_buf, idx + 1);
}

/* ======================== 应答包解析 ======================== */

/**
  * @brief  验证并解析舵机应答包
  *
  * 应答格式: FF F5 ID Len Status Param[0..N-1] CS
  *         Len = ID(1) + Status(1) + N = 2 + N
  *         CS  = ~(ID + Len + Status + Param...) & 0xFF
  *
  * @param  buf 接收缓冲区
  * @param  len 接收到的字节数
  */
static void SERVO_ParseResponse(const uint8_t *buf, uint8_t len)
{
    /* 最小有效应答: FF F5 ID Len Status CS (6 字节) */
    if (len < 6) {
        return;
    }

    /* 验证应答字头 */
    if (buf[0] != FRAME_RESP_HEADER0 || buf[1] != FRAME_RESP_HEADER1) {
        return;
    }

    uint8_t id        = buf[2];
    uint8_t data_len  = buf[3];   /* = ID(1) + Status(1) + Params(N) */
    uint8_t status    = buf[4];
    uint8_t param_len = (data_len >= 2) ? (data_len - 2) : 0;

    /* 验证总长度: 2(header) + data_len + 1(CS) */
    if (len < (uint8_t)(3 + data_len)) {
        return;
    }

    /* 验证校验和: ~(ID + data_len + status + 全部参数) & 0xFF */
    {
        uint16_t sum = id + data_len + status;
        for (uint8_t i = 0; i < param_len; i++) {
            sum += buf[5 + i];
        }
        uint8_t calc_cs = (~((uint8_t)sum)) & 0xFF;
        uint8_t recv_cs = buf[5 + param_len];
        if (calc_cs != recv_cs) {
            return;     /* 校验和不匹配, 丢弃 */
        }
    }

    /* 填充响应结构体 */
    Servo_ResponsePacket resp;
    resp.id        = id;
    resp.status    = status;
    resp.param_len = (param_len < sizeof(resp.params)) ? param_len : sizeof(resp.params);
    for (uint8_t i = 0; i < resp.param_len; i++) {
        resp.params[i] = buf[5 + i];
    }

    /* 调用用户回调 */
    if (user_callback) {
        user_callback(&resp);
    }

    servo_rx_cnt++;
}

/* ======================== 公有 API - 控制 ======================== */

/**
  * @brief  设置舵机目标位置
  *         帧: FF FF ID 07 03 2A posH posL timeH timeL CS
  */
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t run_time)
{
    /* 钳位到有效范围 */
    if (position > SERVO_POS_MAX) {
        position = SERVO_POS_MAX;
    }

    uint8_t params[5] = {
        SERVO_ADDR_TARGET_POS,
        (uint8_t)(position >> 8),       /* 大端: 高字节在前 */
        (uint8_t)(position & 0xFF),
        (uint8_t)(run_time >> 8),
        (uint8_t)(run_time & 0xFF)
    };
    SERVO_SendFrame(id, SERVO_CMD_WRITE, params, 5);
}

/**
  * @brief  使能 / 关闭扭矩
  *         帧: FF FF ID 04 03 28 01/00 CS
  */
void Servo_SetTorque(uint8_t id, bool enable)
{
    uint8_t params[2] = {SERVO_ADDR_TORQUE, enable ? 0x01 : 0x00};
    SERVO_SendFrame(id, SERVO_CMD_WRITE, params, 2);
}

/**
  * @brief  设置舵机/电机模式
  *         帧: FF FF ID 04 03 1C mode CS
  */
void Servo_SetMode(uint8_t id, uint8_t mode)
{
    uint8_t params[2] = {SERVO_ADDR_MODE, mode};
    SERVO_SendFrame(id, SERVO_CMD_WRITE, params, 2);
}

/**
  * @brief  读取当前位置 (异步)
  *         帧: FF FF ID 04 02 38 02 CS
  *         应答参数: posH posL (Uint16, 大端)
  * @note   不可使用广播 ID
  */
void Servo_ReadPosition(uint8_t id)
{
    if (id == SERVO_ID_BROADCAST) {
        return;     /* 协议禁止对广播 ID 使用读指令 */
    }

    uint8_t params[2] = {SERVO_ADDR_CURRENT_POS, 0x02};  /* 地址 0x38, 读 2 字节 */
    SERVO_SendFrame(id, SERVO_CMD_READ, params, 2);
}

/**
  * @brief  查询舵机状态 (PING)
  *         帧: FF FF ID 02 01 CS
  *         应答: 舵机返回状态字节 (无参数)
  * @note   不可使用广播 ID
  */
void Servo_Ping(uint8_t id)
{
    if (id == SERVO_ID_BROADCAST) {
        return;     /* 协议禁止对广播 ID 使用 PING */
    }

    SERVO_SendFrame(id, SERVO_CMD_PING, NULL, 0);
}

/**
  * @brief  恢复出厂设置
  *         帧: FF FF ID 02 06 CS
  */
void Servo_Reset(uint8_t id)
{
    SERVO_SendFrame(id, SERVO_CMD_RESET, NULL, 0);
}

/**
  * @brief  异步写目标位置 (REG WRITE)
  *         帧: FF FF ID 07 04 2A posH posL timeH timeL CS
  *
  *         与 Servo_SetPosition 的区别:
  *         写入后不会立即执行, 而是等待 Servo_Action() 指令触发。
  *         用于多个舵机同时启动。
  * @note   不可使用广播 ID, 仅支持地址 0x2A (目标位置控制)
  */
void Servo_RegWritePosition(uint8_t id, uint16_t position, uint16_t run_time)
{
    if (id == SERVO_ID_BROADCAST) {
        return;     /* 协议限制 REG WRITE 仅可使用舵机 ID */
    }

    /* 钳位到有效范围 */
    if (position > SERVO_POS_MAX) {
        position = SERVO_POS_MAX;
    }

    uint8_t params[5] = {
        SERVO_ADDR_TARGET_POS,
        (uint8_t)(position >> 8),
        (uint8_t)(position & 0xFF),
        (uint8_t)(run_time >> 8),
        (uint8_t)(run_time & 0xFF)
    };
    SERVO_SendFrame(id, SERVO_CMD_REG_WRITE, params, 5);
}

/**
  * @brief  执行异步写 (ACTION)
  *         帧: FF FF FE 02 05 CS
  *
  *         触发所有已接收 REG WRITE 指令的舵机同时开始转动。
  *         使用广播 ID, 舵机不会应答。
  */
void Servo_Action(void)
{
    SERVO_SendFrame(SERVO_ID_BROADCAST, SERVO_CMD_ACTION, NULL, 0);
}

/**
  * @brief  写舵机寄存器 (通用)
  *         帧: FF FF ID Len 03 addr data[0..N-1] CS
  */
void Servo_WriteRegister(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len)
{
    /* 参数 = [addr, data0, data1, ...] */
    uint8_t params[len + 1];
    params[0] = addr;
    for (uint8_t i = 0; i < len; i++) {
        params[1 + i] = data[i];
    }
    SERVO_SendFrame(id, SERVO_CMD_WRITE, params, len + 1);
}

/* ======================== 公有 API - 状态与回调 ======================== */

bool Servo_IsBusy(void)
{
    return servo_busy;
}

void Servo_SetResponseCallback(Servo_ResponseCallback callback)
{
    user_callback = callback;
}

/**
  * @brief  DMA 发送完成回调
  *
  *         在 bsp_uart.c 的 HAL_UART_TxCpltCallback 中调用:
  *         if (huart == &huart7) {
  *             Vofa_Handle.is_busy = false;
  *             Servo_TxCpltCallback();
  *         }
  *
  * @note   只清除发送忙标志, 接收已由 SERVO_SendFrame 提前启动。
  *         这样可以避免 VOFA 发送等非舵机 TX 完成后误启动接收。
  */
void Servo_TxCpltCallback(void)
{
    servo_busy = false;
    servo_tx_ok_cnt++;
}

/**
  * @brief  RX 事件回调
  *
  *         在 bsp_uart.c 的 HAL_UARTEx_RxEventCallback 中调用:
  *         if (huart == &huart7) {
  *             Servo_RxEventCallback(huart, Size);
  *             return;
  *         }
  */
void Servo_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart7) {
        return;
    }

    /* 解析舵机应答包 */
    SERVO_ParseResponse(servo_rx_buf, (uint8_t)size);

    /* 注意: 不在此处重开 RX 接收。
     * 每次发送非广播指令时, SERVO_SendFrame 会启动一次接收。
     * 这种"发送→接收→完毕"的模式与主从问答协议匹配。 */
}

/* ======================== 调试信息 ======================== */

uint32_t Servo_GetTxCount(void)
{
    return servo_tx_count;
}

uint32_t Servo_GetTxOkCount(void)
{
    return servo_tx_ok_cnt;
}

uint32_t Servo_GetRxCount(void)
{
    return servo_rx_cnt;
}

const uint8_t *Servo_GetLastFrame(uint8_t *len)
{
    if (len) *len = servo_last_frame_len;
    return servo_last_frame;
}
