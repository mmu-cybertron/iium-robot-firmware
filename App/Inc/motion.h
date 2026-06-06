#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "robot_types.h"

motor_command_t motion_forward(int16_t pwm);
motor_command_t motion_reverse(int16_t pwm);
motor_command_t motion_rotate_left(int16_t pwm);
motor_command_t motion_rotate_right(int16_t pwm);

#endif /* MOTION_H */
