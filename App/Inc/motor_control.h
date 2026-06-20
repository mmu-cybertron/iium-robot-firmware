#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "robot_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_control_init(void);
void motor_control_set_command(motor_command_t command);
void motor_control_stop(void);
void motor_control_update(void);
void motor_control_set_pwm(int16_t left_pwm, int16_t right_pwm);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
