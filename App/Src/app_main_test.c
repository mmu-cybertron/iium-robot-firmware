#include <stdio.h>
#include <stdint.h>

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "motor_driver.h"
#include "led.h"
#include "main.h"

extern I2C_HandleTypeDef VL53L1__PORT;

/* start module: PC13 goes HIGH when the RC transmitter fires the start signal */
#define SM_Signal_Pin       GPIO_PIN_13
#define SM_Signal_GPIO_Port GPIOC

/* LED-to-sensor mapping:
 *   LED_D6 -> left sensor  (VL53L1__ADDR_LEFT)
 *   LED_D7 -> front sensor (VL53L1__ADDR_FRONT)
 *   LED_D8 -> right sensor (VL53L1__ADDR)
 */

#define OPPONENT_RANGE_MM   600   /* detect within 60 cm */
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

static void i2c_bus_recover_and_reinit(void)
{
    /* A stuck sensor is holding SCL/SDA low.
     * Assert all XSHUT pins LOW to force-reset every sensor — this releases the bus.
     * Then de-init and re-init the I2C peripheral to clear the STM32 side,
     * and re-run the full sensor init sequence. */
    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin | XSHUT_2_Pin | XSHUT_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(8);

    HAL_I2C_DeInit(&VL53L1__PORT);
    HAL_I2C_Init(&VL53L1__PORT);

    VL53L1__InitAll();

    VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
    VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
    VL53L1X_StartRanging(VL53L1__ADDR);
    // HAL_Delay(50);
}

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
    /* wait for the first measurement cycle to complete on all sensors (timing budget = 33 ms) */
    HAL_Delay(50);

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
    HAL_Delay(1000);  /* regulation start delay */
    /* ------------------- */

    while (1) {
        if (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) != GPIO_PIN_SET) {
            motor_driver_brake();
            continue;
        }

        status_left  = VL53L1X_GetDistance(VL53L1__ADDR_LEFT,  &left);
        VL53L1X_ClearInterrupt(VL53L1__ADDR_LEFT);

        status_front = VL53L1X_GetDistance(VL53L1__ADDR_FRONT, &front);
        VL53L1X_ClearInterrupt(VL53L1__ADDR_FRONT);

        status_right = VL53L1X_GetDistance(VL53L1__ADDR,       &right);
        VL53L1X_ClearInterrupt(VL53L1__ADDR);

        if (status_left != 0 || status_front != 0 || status_right != 0) {
            /* one sensor locked the bus — reset all and reinit */
            i2c_bus_recover_and_reinit();
            left = 2000;
            front = 2000;
            right = 2000;
        }

        if (left < 30){
            left = 2000;
        }
        if (right < 30){
            right = 2000;
        }
        if (front < 30){
            front = 2000;
        }
        


        uint8_t see_left  = (left  < OPPONENT_RANGE_MM);
        uint8_t see_front = (front < OPPONENT_RANGE_MM);
        uint8_t see_right = (right < OPPONENT_RANGE_MM);

        /* LED_D6 = left, LED_D7 = front, LED_D8 = right */
        led_write_mask((uint8_t)((see_left  << LED_D6) |
                                 (see_front << LED_D7) |
                                 (see_right << LED_D8)));

        if (see_front) {
            /* opponent dead ahead — charge */
            // led_all_on();
            // motor_driver_brake();
            motor_driver_set_pwm(1650, 1650);
        } else if (see_left) {
            /* opponent to the left — pivot left */
            motor_driver_set_pwm(1750, SPEED_TURN_INNER);
        } else if (see_right) {
            /* opponent to the right — pivot right */
            motor_driver_set_pwm(SPEED_TURN_INNER, 1750);
        } else {
            /* no opponent in range — spin to search */
            motor_driver_brake();
        }
    }
}
