#include "ESP32_LOG.h"
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "usart.h"

#define UART_STATE_SEND_QUEUE_LEN      16U
#define UART_STATE_SEND_TASK_STACK     256U
#define UART_STATE_SEND_TASK_PRIORITY  (tskIDLE_PRIORITY + 2U)
#define UART_STATE_SEND_QUEUE_WAIT_TICK pdMS_TO_TICKS(10U)
#define UART_STATE_SEND_TX_TIMEOUT_MS  20U
#define UART_STATE_SEND_FRAME_MAX_LEN  (2U + 6U + UART_STATE_SEND_MAX_NAME_LEN + UART_STATE_SEND_MAX_DATA_LEN + 2U)

typedef struct {
    uint16_t len;
    uint8_t frame[UART_STATE_SEND_FRAME_MAX_LEN];
} Uart_State_Send_Frame_t;

static QueueHandle_t uart_state_send_queue = NULL;
static TaskHandle_t uart_state_send_task_handle = NULL;
static uint8_t uart_state_send_seq = 0U;

/*********************************************************************************************************
*                                              内部函数
*********************************************************************************************************/
static uint16_t Uart_State_Send_Crc16CcittFalse(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static HAL_StatusTypeDef Uart_State_Send_CheckName(const char *name, uint8_t *name_len)
{
    if (name == NULL || name_len == NULL) {
        return HAL_ERROR;
    }

    size_t len = strlen(name);
    if (len == 0U || len > UART_STATE_SEND_MAX_NAME_LEN) {
        return HAL_ERROR;
    }

    *name_len = (uint8_t)len;
    return HAL_OK;
}

static HAL_StatusTypeDef Uart_State_Send_Enqueue(Uart_State_Send_Type_e type,
                                                 const char *name,
                                                 const uint8_t *data,
                                                 uint16_t data_len)
{
    if (uart_state_send_queue == NULL) {
        return HAL_ERROR;
    }
    if (data_len > UART_STATE_SEND_MAX_DATA_LEN || (data_len > 0U && data == NULL)) {
        return HAL_ERROR;
    }

    uint8_t name_len = 0U;
    if (Uart_State_Send_CheckName(name, &name_len) != HAL_OK) {
        return HAL_ERROR;
    }

    if ((type == UART_STATE_SEND_TYPE_F32 || type == UART_STATE_SEND_TYPE_I32 || type == UART_STATE_SEND_TYPE_U32) &&
        data_len != 4U) {
        return HAL_ERROR;
    }

    Uart_State_Send_Frame_t frame = {0};
    uint16_t index = 0U;

    frame.frame[index++] = 0xA5U;
    frame.frame[index++] = 0x5AU;
    frame.frame[index++] = 0x01U;
    frame.frame[index++] = (uint8_t)type;
    frame.frame[index++] = uart_state_send_seq++;
    frame.frame[index++] = name_len;
    frame.frame[index++] = (uint8_t)(data_len & 0xFFU);
    frame.frame[index++] = (uint8_t)((data_len >> 8U) & 0xFFU);

    memcpy(&frame.frame[index], name, name_len);
    index += name_len;

    if (data_len > 0U) {
        memcpy(&frame.frame[index], data, data_len);
        index += data_len;
    }

    uint16_t crc = Uart_State_Send_Crc16CcittFalse(&frame.frame[2], (uint16_t)(index - 2U));
    frame.frame[index++] = (uint8_t)(crc & 0xFFU);
    frame.frame[index++] = (uint8_t)((crc >> 8U) & 0xFFU);
    frame.len = index;

    if (xQueueSend(uart_state_send_queue, &frame, UART_STATE_SEND_QUEUE_WAIT_TICK) != pdTRUE) {
        return HAL_BUSY;
    }

    return HAL_OK;
}

static void Uart_State_Send_Task(void *argument)
{
    (void)argument;
    Uart_State_Send_Frame_t frame;

    while (1) {
        if (xQueueReceive(uart_state_send_queue, &frame, portMAX_DELAY) == pdTRUE) {
            HAL_UART_Transmit(&huart10, frame.frame, frame.len, UART_STATE_SEND_TX_TIMEOUT_MS);
        }
    }
}

/*********************************************************************************************************
*                                              外部接口
*********************************************************************************************************/
HAL_StatusTypeDef Uart_State_Send_Init(void)
{
    if (uart_state_send_queue == NULL) {
        uart_state_send_queue = xQueueCreate(UART_STATE_SEND_QUEUE_LEN, sizeof(Uart_State_Send_Frame_t));
        if (uart_state_send_queue == NULL) {
            return HAL_ERROR;
        }
    }

    if (uart_state_send_task_handle == NULL) {
        BaseType_t ret = xTaskCreate(Uart_State_Send_Task,
                                     "uart_state_tx",
                                     UART_STATE_SEND_TASK_STACK,
                                     NULL,
                                     UART_STATE_SEND_TASK_PRIORITY,
                                     &uart_state_send_task_handle);
        if (ret != pdPASS) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef Uart_State_Send_Float(const char *name, float value)
{
    uint8_t data[4];
    memcpy(data, &value, sizeof(data));
    return Uart_State_Send_Enqueue(UART_STATE_SEND_TYPE_F32, name, data, sizeof(data));
}

HAL_StatusTypeDef Uart_State_Send_Int32(const char *name, int32_t value)
{
    uint8_t data[4];
    memcpy(data, &value, sizeof(data));
    return Uart_State_Send_Enqueue(UART_STATE_SEND_TYPE_I32, name, data, sizeof(data));
}

HAL_StatusTypeDef Uart_State_Send_Uint32(const char *name, uint32_t value)
{
    uint8_t data[4];
    memcpy(data, &value, sizeof(data));
    return Uart_State_Send_Enqueue(UART_STATE_SEND_TYPE_U32, name, data, sizeof(data));
}

HAL_StatusTypeDef Uart_State_Send_Bytes(const char *name, const uint8_t *data, uint16_t data_len)
{
    return Uart_State_Send_Enqueue(UART_STATE_SEND_TYPE_BYTES, name, data, data_len);
}

HAL_StatusTypeDef Uart_State_Send_Text(const char *name, const char *text)
{
    if (text == NULL) {
        return HAL_ERROR;
    }

    size_t len = strlen(text);
    if (len > UART_STATE_SEND_MAX_DATA_LEN) {
        return HAL_ERROR;
    }

    return Uart_State_Send_Enqueue(UART_STATE_SEND_TYPE_TEXT, name, (const uint8_t *)text, (uint16_t)len);
}
