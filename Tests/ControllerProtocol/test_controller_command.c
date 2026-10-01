#include "custom_controller_protocol.h"
#include "Minipc.h"
#include "usb_comm.h"
#include "usbd_cdc_if.h"
#include "CRC.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t test_primask;
USBD_HandleTypeDef hUsbDeviceHS;
static unsigned int new_frames;
static unsigned int legacy_frames;

uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length)
{
    assert(data[0] == 0xAAU && length == 27U);
    return USBD_OK;
}

static void frame_received(const uint8_t *frame, size_t length, void *context)
{
    (void)context;
    if (length == CONTROLLER_COMMAND_SIZE) {
        ControllerCommand command;
        assert(ControllerCommand_Decode(frame, length, &command));
        assert(command.sequence == 7U && command.timestamp_ms == 1234U);
        assert(command.positions[2] == -2.0f && command.positions[5] == 5.0f);
        assert(command.buttons == 0x1234U && command.valid_joint_mask == 0x15U);
        assert(command.joystick_x == 2000U && command.joystick_y == 2100U);
        assert(command.mouse[0] == -1 && command.mouse[2] == -3);
        assert(command.keys == 0x2AAAAU && command.main_buttons == 4U && command.handle_buttons == 8U);
        ++new_frames;
    } else {
        assert(length == 60U);
        ++legacy_frames;
    }
}

static void flush_stream(void)
{
    USBComm_ResetFromISR();
    MiniPC_Receive_Info();
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    FILE *input = fopen(argv[1], "rb");
    assert(input != NULL);
    uint8_t frame[CONTROLLER_COMMAND_SIZE];
    assert(fread(frame, 1U, sizeof(frame), input) == sizeof(frame));
    fclose(input);
    for (size_t split = 0U; split <= sizeof(frame); ++split) {
        ControllerStreamParser parser = {0};
        new_frames = 0U;
        ControllerStream_Feed(&parser, frame, split, frame_received, NULL);
        ControllerStream_Feed(&parser, frame + split, sizeof(frame) - split, frame_received, NULL);
        assert(new_frames == 1U && parser.used == 0U);
    }
    ControllerStreamParser parser = {0};
    uint8_t corrupted[sizeof(frame)];
    memcpy(corrupted, frame, sizeof(frame));
    corrupted[20] ^= 1U;
    new_frames = 0U;
    ControllerStream_Feed(&parser, corrupted, sizeof(corrupted), frame_received, NULL);
    ControllerStream_Feed(&parser, frame, sizeof(frame), frame_received, NULL);
    assert(new_frames == 1U);
    new_frames = 0U;
    ControllerStream_Feed(&parser, frame, sizeof(frame) - 5U, frame_received, NULL);
    ControllerStream_Feed(&parser, frame, sizeof(frame), frame_received, NULL);
    assert(new_frames == 1U);
    uint8_t legacy[60] = {0};
    legacy[0] = 0xAAU;
    legacy[59] = 0x55U;
    legacy[27] = legacy[29] = 0U;
    legacy[28] = legacy[30] = 8U;
    legacy[58] = 16U;
    ControllerStream_Feed(&parser, legacy, sizeof(legacy), frame_received, NULL);
    ControllerStream_Feed(&parser, frame, sizeof(frame), frame_received, NULL);
    assert(legacy_frames == 1U && new_frames == 2U);

    flush_stream();
    const float initial_j2 = MiniPC_Data.joint_pos_data[1];
    USBComm_PushFromISR(frame, 17U);
    MiniPC_Receive_Info();
    assert(MiniPC_Data.lost);
    USBComm_PushFromISR(frame + 17, sizeof(frame) - 17U);
    MiniPC_Receive_Info();
    assert(!MiniPC_Data.lost && MiniPC_Data.command_sequence == 7U);
    assert(MiniPC_Data.joint_pos_data[1] == initial_j2);
    assert(MiniPC_Data.joint_pos_data[2] == -2.0f && MiniPC_Data.joint_pos_data[4] == 4.0f);
    assert(MiniPC_Data.btn4 == 1U && MiniPC_Data.key_s == 1U && MiniPC_Data.key_w == 0U);
    assert(MiniPC_Data.mouse_x == -1 && MiniPC_Data.controller_buttons == 0x1234U);

    USBComm_PushFromISR(frame, 17U);
    MiniPC_Receive_Info();
    flush_stream();
    USBComm_PushFromISR(frame, sizeof(frame));
    MiniPC_Receive_Info();
    assert(MiniPC_Data.command_sequence == 7U);
    USBComm_PushFromISR(legacy, sizeof(legacy));
    MiniPC_Receive_Info();
    assert(MiniPC_Data.valid_joint_mask == 0x3FU && MiniPC_Data.joint_pos_data[2] == 0.0f);

    uint8_t overflow[1100] = {0};
    USBComm_PushFromISR(overflow, sizeof(overflow));
    MiniPC_Receive_Info();
    assert(USBComm_GetDroppedBytes() != 0U);
    USBComm_PushFromISR(frame, sizeof(frame));
    MiniPC_Receive_Info();
    assert(MiniPC_Data.joint_pos_data[2] == -2.0f);

    memcpy(corrupted, frame, sizeof(frame));
    corrupted[53] = 0U;
    Append_CRC16_Check_Sum(corrupted, sizeof(corrupted));
    USBComm_PushFromISR(corrupted, sizeof(corrupted));
    MiniPC_Receive_Info();
    assert(MiniPC_Data.joystick_x == 2048U && MiniPC_Data.key_s == 0U && MiniPC_Data.btn4 == 0U);
    assert(MiniPC_Data.controller_buttons == 0x1234U);

    float positions[6] = {0};
    assert(MiniPC_Transmit_Info(positions, 6U) == USBD_FAIL);
    USBD_CDC_HandleTypeDef cdc = {.TxState = 1U};
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceHS.pClassData = &cdc;
    assert(MiniPC_Transmit_Info(positions, 6U) == USBD_BUSY);
    cdc.TxState = 0U;
    assert(MiniPC_Transmit_Info(positions, 6U) == USBD_OK);
    puts("Controller command integration tests passed");
    return 0;
}
