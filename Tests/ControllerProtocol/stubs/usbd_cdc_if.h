#ifndef TEST_CDC_IF_H
#define TEST_CDC_IF_H
#include <stdint.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { uint8_t dev_state; void *pClassData; } USBD_HandleTypeDef;
uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length);
#endif
