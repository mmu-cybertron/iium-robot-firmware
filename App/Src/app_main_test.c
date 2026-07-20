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

uint8_t status_left=0;
uint8_t status_front =0;
uint8_t status_right = 0;


/* LED-to-sensor mapping:
 *   LED_D6 -> left sensor  (VL53L1__ADDR_LEFT)
 *   LED_D7 -> front sensor (VL53L1__ADDR_FRONT)
 *   LED_D8 -> right sensor (VL53L1__ADDR)
 */

static void vl53_fail_and_halt(led_id_t failed_led)
{
    motor_driver_brake();

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < VL53_FAIL_BLINK_MS) {
        led_on(failed_led);
        HAL_Delay(VL53_BLINK_PERIOD_MS);
        led_off(failed_led);
        HAL_Delay(VL53_BLINK_PERIOD_MS);
    }

    /* halt with the failed sensor's LED solid on */
    while (1) {
        led_on(failed_led);
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
        /* all three blink — unknown which sensor failed at init */
        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < VL53_FAIL_BLINK_MS) {
            led_all_on();
            HAL_Delay(VL53_BLINK_PERIOD_MS);
            led_all_off();
            HAL_Delay(VL53_BLINK_PERIOD_MS);
        }
        while (1) { led_all_on(); }
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

        status_left  = VL53L1X_GetDistance(VL53L1__ADDR_LEFT, &left);
        status_front = VL53L1X_GetDistance(VL53L1__ADDR_FRONT, &front);
        status_right = VL53L1X_GetDistance(VL53L1__ADDR, &right);

        if (status_left != 0) {
            vl53_fail_and_halt(LED_D6);  /* LED_D6 = left sensor */
        }

        if (status_front != 0) {
            vl53_fail_and_halt(LED_D7);  /* LED_D7 = front sensor */
        }

        if (status_right != 0) {
            vl53_fail_and_halt(LED_D8);  /* LED_D8 = right sensor */
        }
    }
}
