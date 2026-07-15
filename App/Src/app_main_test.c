#include <stdio.h>
#include <stdint.h>

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "motor_driver.h"
#include "led.h"

#define MOTOR_PWM_HIGH  2200
#define MOTOR_PWM_LOW   1000
#define SWITCH_INTERVAL_MS 1000
#define VL53_FAIL_BLINK_MS 10000
#define VL53_BLINK_PERIOD_MS 100

uint8_t  status;
uint16_t left = 0, front = 0, right = 0, rr = 0, rl = 0;

uint32_t fail_left  = 0;
uint32_t fail_front = 0;
uint32_t fail_right = 0;

static void vl53_fail_and_halt(void)
{
    printf("[VL53] Communication failure — blinking for %u ms then halting\r\n", VL53_FAIL_BLINK_MS);
    motor_driver_brake();

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < VL53_FAIL_BLINK_MS) {
        led_all_on();
        HAL_Delay(VL53_BLINK_PERIOD_MS);
        led_all_off();
        HAL_Delay(VL53_BLINK_PERIOD_MS);
    }

    /* halt */
    while (1) {
        led_all_off();
    }
}

void app_main_test(void)
{
    led_init();
    motor_driver_init();
    HAL_Delay(500);

    status = VL53L1__InitAll();
    if (status != 0) {
        printf("[VL53L1] Init failed (status=%u)\r\n", (unsigned int)status);
        vl53_fail_and_halt();
    } else {
        printf("[VL53L1] Init OK\r\n");
    }

    VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
    VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
    VL53L1X_StartRanging(VL53L1__ADDR);

    uint32_t last_switch = HAL_GetTick();
    int16_t current_pwm = MOTOR_PWM_HIGH;

    while (1) {
        /* alternate motor direction every SWITCH_INTERVAL_MS */
        if (HAL_GetTick() - last_switch >= SWITCH_INTERVAL_MS) {
            last_switch = HAL_GetTick();
            current_pwm = (current_pwm == MOTOR_PWM_HIGH) ? MOTOR_PWM_LOW : MOTOR_PWM_HIGH;
        }
        motor_driver_set_pwm(current_pwm, current_pwm);

        uint8_t failure_mask = VL53L1__ReadAll(&left, &front, &right, &rr, &rl);

        if (failure_mask & 0x01) fail_left++;
        if (failure_mask & 0x02) fail_front++;
        if (failure_mask & 0x04) fail_right++;

        if (failure_mask != 0) {

            vl53_fail_and_halt();
        }
    }
}
