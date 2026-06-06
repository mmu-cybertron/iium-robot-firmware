#include "motion.h"

motor_command_t motion_forward(int16_t pwm)
{
    motor_command_t command = { pwm, pwm };
    return command;
}

motor_command_t motion_reverse(int16_t pwm)
{
    motor_command_t command = { (int16_t)-pwm, (int16_t)-pwm };
    return command;
}

motor_command_t motion_rotate_left(int16_t pwm)
{
    motor_command_t command = { (int16_t)-pwm, pwm };
    return command;
}

motor_command_t motion_rotate_right(int16_t pwm)
{
    motor_command_t command = { pwm, (int16_t)-pwm };
    return command;
}
