#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_START_DELAY,
    ROBOT_STATE_SEARCH,
    ROBOT_STATE_ATTACK,
    ROBOT_STATE_EDGE_ESCAPE,
    ROBOT_STATE_RECOVER,
    ROBOT_STATE_FAULT
} robot_state_t;

typedef enum {
    DIRECTION_STOP = 0,
    DIRECTION_FORWARD,
    DIRECTION_REVERSE,
    DIRECTION_LEFT,
    DIRECTION_RIGHT
} robot_direction_t;

typedef struct {
    int16_t left_pwm;
    int16_t right_pwm;
} motor_command_t;

typedef struct {
    uint8_t front_left;
    uint8_t front_right;
    uint8_t rear_left;
    uint8_t rear_right;
} edge_status_t;

typedef struct {
    uint8_t front;
    uint8_t left;
    uint8_t right;
    uint16_t distance_mm;
} opponent_status_t;

#endif /* ROBOT_TYPES_H */
