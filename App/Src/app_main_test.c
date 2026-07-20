#include <stdio.h>
#include <stdint.h>

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "motor_driver.h"
#include "led.h"
#include "main.h"

/* start module: PC13 goes HIGH when the RC transmitter fires the start signal */
#define SM_Signal_Pin       GPIO_PIN_13
#define SM_Signal_GPIO_Port GPIOC

/* LED-to-sensor mapping:
 *   LED_D6 -> left sensor  (VL53L1__ADDR_LEFT)
 *   LED_D7 -> front sensor (VL53L1__ADDR_FRONT)
 *   LED_D8 -> right sensor (VL53L1__ADDR)
 */

#define OPPONENT_RANGE_MM   500   /* detect within 50 cm */
#define SPEED_MAX           1750  /* full attack speed */
#define SPEED_TURN_INNER    1250  /* inner wheel during turn */
#define SPEED_SEARCH        1625  /* spin-in-place speed while searching */
#define VL53_FAIL_BLINK_MS  10000
#define VL53_BLINK_PERIOD_MS 100

uint8_t  status;
uint16_t left = 0, front = 0, right = 0;

uint8_t status_left  = 0;
uint8_t status_front = 0;
uint8_t status_right = 0;

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

    while (1) { led_on(failed_led); }
}

void app_main_test(void)
{
    led_init();
    motor_driver_init();
    HAL_Delay(500);

    status = VL53L1__InitAll();
    if (status != 0) {
        printf("[VL53L1] Init failed (status=%u)\r\n", (unsigned int)status);
        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < VL53_FAIL_BLINK_MS) {
            led_all_on();
            HAL_Delay(VL53_BLINK_PERIOD_MS);
            led_all_off();
            HAL_Delay(VL53_BLINK_PERIOD_MS);
        }
        while (1) { led_all_on(); }
    }

    VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
    VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
    VL53L1X_StartRanging(VL53L1__ADDR);

    /* --- start module --- */
    /* blink all LEDs slowly while waiting for the start signal */
    printf("[START] Waiting for start signal (PC13 HIGH)...\r\n");
    while (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) != GPIO_PIN_SET) {
        led_all_on();
        HAL_Delay(200);
        led_all_off();
        HAL_Delay(200);
    }
    printf("[START] Signal received. Waiting 5 s before moving...\r\n");
    led_all_off();
    HAL_Delay(5000);  /* regulation start delay */
    /* ------------------- */

    while (1) {
        if (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) != GPIO_PIN_SET) {
            motor_driver_brake();
            continue;
        }

        status_left  = VL53L1X_GetDistance(VL53L1__ADDR_LEFT,  &left);
        status_front = VL53L1X_GetDistance(VL53L1__ADDR_FRONT, &front);
        status_right = VL53L1X_GetDistance(VL53L1__ADDR,       &right);

        if (status_left  != 0) { vl53_fail_and_halt(LED_D6); }  /* LED_D6 = left sensor  */
        if (status_front != 0) { vl53_fail_and_halt(LED_D7); }  /* LED_D7 = front sensor */
        if (status_right != 0) { vl53_fail_and_halt(LED_D8); }  /* LED_D8 = right sensor */

        uint8_t see_left  = (left  < OPPONENT_RANGE_MM);
        uint8_t see_front = (front < OPPONENT_RANGE_MM);
        uint8_t see_right = (right < OPPONENT_RANGE_MM);

        if (see_front) {
            /* opponent dead ahead — charge */
            motor_driver_set_pwm(SPEED_MAX, SPEED_MAX);
        } else if (see_left) {
            /* opponent to the left — pivot left */
            motor_driver_set_pwm(SPEED_TURN_INNER, SPEED_MAX);
        } else if (see_right) {
            /* opponent to the right — pivot right */
            motor_driver_set_pwm(SPEED_MAX, SPEED_TURN_INNER);
        } else {
            /* no opponent in range — spin to search */
            motor_driver_set_pwm(SPEED_SEARCH, SPEED_TURN_INNER);
        }
    }
}
