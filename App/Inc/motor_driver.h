#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void motor_driver_init(void);
void motor_driver_set_pwm(int16_t left_pwm, int16_t right_pwm);
void motor_driver_brake(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
