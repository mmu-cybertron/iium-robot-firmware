#include "robot.h"
#include "robot_config.h"
#include "main.h"
#include "usart1_log.h"
#include "vl53.h"

extern I2C_HandleTypeDef hi2c1;

#define TOF_LOG_PERIOD_MS  500U

static uint8_t vl53_initialized = 0U;

void robot_init(void)
{
    HAL_StatusTypeDef status;

    LOG_PRINT("VL53L0X test starting\r\n");

    // Same pattern as the main.c reference
    set_vl53_i2c_handler(&hi2c1);   // reference used hi2c2, yours is hi2c1
    status = vl53_init_multi();

    if (status != HAL_OK) {
        LOG_PRINT("[VL53] Failed to initialize VL53L0X\r\n");
        vl53_initialized = 0U;
        return;
    }

    vl53_initialized = 1U;
    LOG_PRINT("[VL53] Ready\r\n");
}

void robot_update(void)
{
    static uint32_t last_log_ms = 0U;
    const uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_log_ms) < TOF_LOG_PERIOD_MS) {
        return;
    }
    last_log_ms = now_ms;

    if (vl53_initialized == 0U) {
        LOG_PRINT("VL53L0X not initialized\r\n");
        return;
    }

    uint16_t distance = readRangeContinuousMillimeters();
    LOG_PRINT("[VL53L0X] Distance: %u mm\r\n", distance);
}

void robot_background(void)
{
}
