#include "motor_driver.h"

static int16_t last_left_pwm;
static int16_t last_right_pwm;

void motor_driver_init(void)
{
    last_left_pwm = 0;
    last_right_pwm = 0;
}

void motor_driver_set_pwm(int16_t left_pwm, int16_t right_pwm)
{
    last_left_pwm = left_pwm;
    last_right_pwm = right_pwm;

    /*
     * TODO: Map signed PWM to CubeMX timer channels and direction GPIOs.
     * Keep HAL calls here instead of inside strategy or control modules.
     */
}

void motor_driver_brake(void)
{
    last_left_pwm = 0;
    last_right_pwm = 0;
}
