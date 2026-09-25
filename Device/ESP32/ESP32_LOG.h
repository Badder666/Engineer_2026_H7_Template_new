#ifndef UART_STATE_SEND_H
#define UART_STATE_SEND_H

#include <stddef.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_STATE_SEND_MAX_NAME_LEN 15U
#define UART_STATE_SEND_MAX_DATA_LEN 32U

typedef enum {
    UART_STATE_SEND_TYPE_F32    = 0x01U,
    UART_STATE_SEND_TYPE_I32    = 0x02U,
    UART_STATE_SEND_TYPE_U32    = 0x03U,
    UART_STATE_SEND_TYPE_BYTES  = 0x10U,
    UART_STATE_SEND_TYPE_TEXT   = 0x11U,
} Uart_State_Send_Type_e;

HAL_StatusTypeDef Uart_State_Send_Init(void);
HAL_StatusTypeDef Uart_State_Send_Float(const char *name, float value);
HAL_StatusTypeDef Uart_State_Send_Int32(const char *name, int32_t value);
HAL_StatusTypeDef Uart_State_Send_Uint32(const char *name, uint32_t value);
HAL_StatusTypeDef Uart_State_Send_Bytes(const char *name, const uint8_t *data, uint16_t data_len);
HAL_StatusTypeDef Uart_State_Send_Text(const char *name, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* UART_STATE_SEND_H */
