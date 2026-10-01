#ifndef CUSTOM_CONTROLLER_PROTOCOL_H
#define CUSTOM_CONTROLLER_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CONTROLLER_COMMAND_SIZE 64U
#define CONTROLLER_COMMAND_TYPE 0x10U
#define CONTROLLER_OPERATOR_VALID 0x01U

typedef struct {
    uint16_t sequence;
    uint32_t timestamp_ms;
    float positions[6];
    uint16_t buttons;
    uint16_t valid_joint_mask;
    uint16_t joystick_x;
    uint16_t joystick_y;
    int16_t mouse[3];
    uint8_t mouse_buttons[3];
    uint8_t flags;
    uint32_t keys;
    uint8_t main_buttons;
    uint8_t handle_buttons;
} ControllerCommand;

typedef void (*ControllerFrameHandler)(const uint8_t *frame, size_t length, void *context);

typedef struct {
    uint8_t buffer[CONTROLLER_COMMAND_SIZE];
    size_t used;
} ControllerStreamParser;

bool ControllerCommand_Decode(const uint8_t *frame, size_t length, ControllerCommand *command);
/* Accepts new CRC frames and legacy AA...55 frames on the same stream. */
void ControllerStream_Feed(ControllerStreamParser *parser, const uint8_t *data,
                           size_t length, ControllerFrameHandler handler, void *context);

#endif
