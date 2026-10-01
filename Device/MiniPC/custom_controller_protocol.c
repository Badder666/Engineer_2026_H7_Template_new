#include "custom_controller_protocol.h"
#include "CRC.h"
#include <math.h>
#include <string.h>

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

bool ControllerCommand_Decode(const uint8_t *frame, size_t length, ControllerCommand *command)
{
    if (frame == NULL || command == NULL || length != CONTROLLER_COMMAND_SIZE ||
        frame[0] != 0xA5U || frame[1] != 0x5AU || frame[2] != 1U ||
        frame[3] != CONTROLLER_COMMAND_TYPE || read_u16(frame + 4) != length ||
        read_u16(frame + 62) != Get_CRC16_Check_Sum((uint8_t *)frame, 62U, 0xFFFFU)) {
        return false;
    }

    ControllerCommand value = {0};
    value.sequence = read_u16(frame + 6);
    value.timestamp_ms = read_u32(frame + 8);
    for (size_t i = 0; i < 6U; ++i) {
        uint32_t bits = read_u32(frame + 12U + 4U * i);
        memcpy(&value.positions[i], &bits, sizeof(bits));
        if (!isfinite(value.positions[i])) {
            return false;
        }
    }
    value.buttons = read_u16(frame + 36);
    value.valid_joint_mask = read_u16(frame + 38);
    value.joystick_x = read_u16(frame + 40);
    value.joystick_y = read_u16(frame + 42);
    for (size_t i = 0; i < 3U; ++i) {
        uint16_t bits = read_u16(frame + 44U + 2U * i);
        memcpy(&value.mouse[i], &bits, sizeof(bits));
        value.mouse_buttons[i] = frame[50U + i];
        if (value.mouse_buttons[i] > 3U) {
            return false;
        }
    }
    value.flags = frame[53];
    value.keys = read_u32(frame + 54);
    value.main_buttons = frame[58];
    value.handle_buttons = frame[59];
    if ((value.valid_joint_mask & ~0x3FU) != 0U || value.joystick_x > 4095U ||
        value.joystick_y > 4095U || (value.flags & ~CONTROLLER_OPERATOR_VALID) != 0U ||
        (value.keys & ~0x3FFFFU) != 0U || frame[60] != 0U || frame[61] != 0U) {
        return false;
    }
    *command = value;
    return true;
}

static void discard(ControllerStreamParser *parser, size_t length)
{
    parser->used -= length;
    memmove(parser->buffer, parser->buffer + length, parser->used);
}

void ControllerStream_Feed(ControllerStreamParser *parser, const uint8_t *data,
                           size_t length, ControllerFrameHandler handler, void *context)
{
    if (parser == NULL || data == NULL || handler == NULL) {
        return;
    }
    for (size_t i = 0; i < length; ++i) {
        parser->buffer[parser->used++] = data[i];
        while (parser->used != 0U) {
            size_t expected;
            bool valid;
            if (parser->buffer[0] == 0xA5U) {
                if (parser->used < 2U) {
                    break;
                }
                if (parser->buffer[1] != 0x5AU) {
                    discard(parser, 1U);
                    continue;
                }
                if (parser->used < 6U) {
                    break;
                }
                if (parser->buffer[2] != 1U || parser->buffer[3] != CONTROLLER_COMMAND_TYPE ||
                    read_u16(parser->buffer + 4) != CONTROLLER_COMMAND_SIZE) {
                    discard(parser, 1U);
                    continue;
                }
                expected = CONTROLLER_COMMAND_SIZE;
                if (parser->used < expected) {
                    break;
                }
                ControllerCommand command;
                valid = ControllerCommand_Decode(parser->buffer, expected, &command);
            } else if (parser->buffer[0] == 0xAAU) {
                expected = 60U;
                if (parser->used < expected) {
                    break;
                }
                uint8_t sum = 0U;
                for (size_t j = 1U; j < 58U; ++j) {
                    sum = (uint8_t)(sum + parser->buffer[j]);
                }
                valid = parser->buffer[58] == sum && parser->buffer[59] == 0x55U;
            } else {
                discard(parser, 1U);
                continue;
            }
            if (valid) {
                handler(parser->buffer, expected, context);
                discard(parser, expected);
            } else {
                discard(parser, 1U);
            }
        }
    }
}
