#include "motor_control.h"

#include "motor_driver.h"
#include "robot_config.h"

static motor_command_t current_command;

static int16_t clamp_pwm(int16_t value)
{
    if (value > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }

    if (value < MOTOR_PWM_MIN) {
        return MOTOR_PWM_MIN;
    }

    return value;
}

void motor_control_init(void)
{
    // current_command.left_pwm = 0;
    // current_command.right_pwm = 0;
    motor_driver_init();
}

void motor_control_set_command(motor_command_t command)
{
    current_command.left_pwm = clamp_pwm(command.left_pwm);
    current_command.right_pwm = clamp_pwm(command.right_pwm);
}

void motor_control_stop(void)
{
    motor_driver_brake();
}

void motor_control_update(void)
{
    motor_driver_set_pwm(current_command.left_pwm, current_command.right_pwm);
}

void motor_control_set_pwm(int16_t left_pwm, int16_t right_pwm)
{
    motor_driver_set_pwm(left_pwm, right_pwm);
}
