#include <stdio.h>
#include <stdint.h>

#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "motor_driver.h"

uint8_t  status;
uint16_t left = 0, front = 0, right = 0, rr = 0, rl = 0;

int16_t left_pwm, right_pwm;

uint32_t fail_left  = 0;
uint32_t fail_front = 0;
uint32_t fail_right = 0;

void app_main_test(void)
{
    left_pwm = 1500;
    right_pwm = 1500;
    motor_driver_init();
    HAL_Delay(500);

    status = VL53L1__InitAll();
    if (status != 0) {
        printf("[VL53L1] Init failed (status=%u)\r\n", (unsigned int)status);
    } else {
        printf("[VL53L1] Init OK\r\n");
    }

    VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
    VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
    VL53L1X_StartRanging(VL53L1__ADDR);

    while (1) {
        motor_driver_set_pwm(left_pwm, right_pwm);
        uint8_t failure_mask = VL53L1__ReadAll(&left, &front, &right, &rr, &rl);

        if (failure_mask & 0x01) fail_left++;
        if (failure_mask & 0x02) fail_front++;
        if (failure_mask & 0x04) fail_right++;
    }
}
