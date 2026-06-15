#include "motor_driver.h"

#include "main.h"
#include "robot_config.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define LEFT_MOTOR_PWM_TIMER      (&htim2)
#define LEFT_MOTOR_PWM_CHANNEL    TIM_CHANNEL_1
#define LEFT_MOTOR_DIR_PORT       LEFT_DIR_GPIO_Port
#define LEFT_MOTOR_DIR_PIN        LEFT_DIR_Pin

#define RIGHT_MOTOR_PWM_TIMER     (&htim3)
#define RIGHT_MOTOR_PWM_CHANNEL   TIM_CHANNEL_2
#define RIGHT_MOTOR_DIR_PORT      RIGHT_DIR_GPIO_Port
#define RIGHT_MOTOR_DIR_PIN       RIGHT_DIR_Pin

#define MOTOR_FORWARD_STATE       GPIO_PIN_RESET
#define MOTOR_REVERSE_STATE       GPIO_PIN_SET

static uint8_t is_initialized;

static int16_t clamp_pwm(int16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }

    if (pwm < MOTOR_PWM_MIN) {
        return MOTOR_PWM_MIN;
    }

    return pwm;
}

static uint32_t pwm_magnitude(int16_t pwm)
{
    if (pwm <= MOTOR_PWM_MIN) {
        return (uint32_t)MOTOR_PWM_MAX;
    }

    return (pwm < 0) ? (uint32_t)-pwm : (uint32_t)pwm;
}

static void set_motor_pwm(TIM_HandleTypeDef *timer,
                          uint32_t channel,
                          int16_t pwm)
{
    pwm = clamp_pwm(pwm);

    __HAL_TIM_SET_COMPARE(timer, channel, pwm_magnitude(pwm));
}

static void stop_pwm_outputs(void)
{
    __HAL_TIM_SET_COMPARE(LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, 0U);
}

static void set_neutral_pwm(void)
{
    __HAL_TIM_SET_COMPARE(LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, MOTOR_PWM_NEUTRAL);
    __HAL_TIM_SET_COMPARE(RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, MOTOR_PWM_NEUTRAL);
}

void motor_driver_init(void)
{
    is_initialized = 0U;
    stop_pwm_outputs();

    if (HAL_TIM_PWM_Start(LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL) != HAL_OK) {
        return;
    }

    if (HAL_TIM_PWM_Start(RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL) != HAL_OK) {
        HAL_TIM_PWM_Stop(LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL);
        return;
    }

    set_neutral_pwm();
    
    is_initialized = 1U;
}

void motor_driver_set_pwm(int16_t left_pwm, int16_t right_pwm)
{
    if (is_initialized == 0U) {
        return;
    }

    set_motor_pwm(LEFT_MOTOR_PWM_TIMER,
                  LEFT_MOTOR_PWM_CHANNEL,
                  left_pwm);
    set_motor_pwm(RIGHT_MOTOR_PWM_TIMER,
                  RIGHT_MOTOR_PWM_CHANNEL,
                  right_pwm);
}

void motor_driver_brake(void)
{
    if (is_initialized == 0U) {
        return;
    }

    __HAL_TIM_SET_COMPARE(LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, MOTOR_PWM_NEUTRAL);
    __HAL_TIM_SET_COMPARE(RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, MOTOR_PWM_NEUTRAL);

}
