#include "robot.h"
#include "robot_config.h"
#include "main.h"
#include "usart1_log.h"
#include "vl53.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"

extern I2C_HandleTypeDef hi2c1;

#define TOF_LOG_PERIOD_MS  500U

static uint8_t vl53_initialized = 0U;
static uint8_t vl53l1_initialized = 0U;

void robot_init(void)
{
    HAL_StatusTypeDef status;

    LOG_PRINT("\r\n=================================\r\n");
    LOG_PRINT("Initializing All 5 ToF Sensors...\r\n");
    LOG_PRINT("=================================\r\n");

    // ==============================================================
    // 1. Initialize the 2 VL53L0X sensors (XSHUT4, XSHUT5)
    // ==============================================================
    set_vl53_i2c_handler(&hi2c1);
    status = vl53_init_multi();

    if (status != HAL_OK) {
        LOG_PRINT("[VL53L0X] Failed to initialize\r\n");
        vl53_initialized = 0U;
    } else {
        vl53_initialized = 1U;
        LOG_PRINT("[VL53L0X] Ready\r\n");
    }


    // ==============================================================
    // 2. Initialize the 3 VL53L1X sensors Manually for Robustness
    // ==============================================================
    LOG_PRINT("\r\n[VL53L1X] Powering down L1X sensors...\r\n");
    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin | XSHUT_2_Pin | XSHUT_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);

    uint8_t err;
    uint8_t success_count = 0;

    // --- L1X LEFT (XSHUT1) ---
    LOG_PRINT("[VL53L1X] Waking LEFT (XSHUT1)...\r\n");
    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin, GPIO_PIN_SET);
    HAL_Delay(20); // Safe 20ms boot time

    err = VL53L1X_SetI2CAddress(0x52, VL53L1__ADDR_LEFT);
    if (err) LOG_PRINT(" -> SetI2CAddress Failed! Code: %d\r\n", err);

    err = VL53L1X_SensorInit(VL53L1__ADDR_LEFT);
    if (err) LOG_PRINT(" -> SensorInit Failed! Code: %d\r\n", err);
    else {
        VL53L1X_SetDistanceMode(VL53L1__ADDR_LEFT, VL53L1__DISTANCE_MODE);
        VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR_LEFT, VL53L1__TIMING_BUDGET);
        VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR_LEFT, VL53L1__INTERMEASUREMENT);
        VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
        LOG_PRINT(" -> LEFT Ready! (Addr: 0x%02X)\r\n", VL53L1__ADDR_LEFT);
        success_count++;
    }

    // --- L1X FRONT (XSHUT2) ---
    LOG_PRINT("[VL53L1X] Waking FRONT (XSHUT2)...\r\n");
    HAL_GPIO_WritePin(GPIOB, XSHUT_2_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    err = VL53L1X_SetI2CAddress(0x52, VL53L1__ADDR_FRONT);
    if (err) LOG_PRINT(" -> SetI2CAddress Failed! Code: %d\r\n", err);

    err = VL53L1X_SensorInit(VL53L1__ADDR_FRONT);
    if (err) LOG_PRINT(" -> SensorInit Failed! Code: %d\r\n", err);
    else {
        VL53L1X_SetDistanceMode(VL53L1__ADDR_FRONT, VL53L1__DISTANCE_MODE);
        VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR_FRONT, VL53L1__TIMING_BUDGET);
        VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR_FRONT, VL53L1__INTERMEASUREMENT);
        VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
        LOG_PRINT(" -> FRONT Ready! (Addr: 0x%02X)\r\n", VL53L1__ADDR_FRONT);
        success_count++;
    }

    // --- L1X RIGHT (XSHUT3) ---
    LOG_PRINT("[VL53L1X] Waking RIGHT (XSHUT3)...\r\n");
    HAL_GPIO_WritePin(GPIOB, XSHUT_3_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    // RIGHT sensor stays at default address 0x52
    err = VL53L1X_SensorInit(VL53L1__ADDR);
    if (err) LOG_PRINT(" -> SensorInit Failed! Code: %d\r\n", err);
    else {
        VL53L1X_SetDistanceMode(VL53L1__ADDR, VL53L1__DISTANCE_MODE);
        VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR, VL53L1__TIMING_BUDGET);
        VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR, VL53L1__INTERMEASUREMENT);
        VL53L1X_StartRanging(VL53L1__ADDR);
        LOG_PRINT(" -> RIGHT Ready! (Addr: 0x%02X)\r\n", VL53L1__ADDR);
        success_count++;
    }

    if (success_count > 0) {
        vl53l1_initialized = 1U;
        LOG_PRINT("\r\n[VL53L1X] Startup Complete!\r\n");
    } else {
        vl53l1_initialized = 0U;
    }
}

void robot_update(void)
{
    static uint32_t last_log_ms = 0U;
    const uint32_t now_ms = HAL_GetTick();

    // The TOF_LOG_PERIOD_MS limits how fast it prints to the terminal!
    if ((now_ms - last_log_ms) < TOF_LOG_PERIOD_MS) {
        return;
    }
    last_log_ms = now_ms;

    uint16_t dist_l0x[2] = {0, 0};
    uint16_t dist_l1x_left = 0, dist_l1x_front = 0, dist_l1x_right = 0;
    uint16_t dummy_rear_right = 0, dummy_rear_left = 0;

    // Read VL53L0X (XSHUT4, XSHUT5)
    if (vl53_initialized == 1U) {
        vl53_read_multi(dist_l0x);
    }

    // Read VL53L1X (XSHUT1, XSHUT2, XSHUT3)
    if (vl53l1_initialized == 1U) {
        VL53L1__ReadAll(&dist_l1x_left, &dist_l1x_front, &dist_l1x_right, &dummy_rear_right, &dummy_rear_left);
    }

    // Print all 5 distances together!
    LOG_PRINT("L1X (1: %4u | 2: %4u | 3: %4u) mm || L0X (4: %4u | 5: %4u) mm\r\n",
              dist_l1x_left, dist_l1x_front, dist_l1x_right, dist_l0x[0], dist_l0x[1]);
}

void robot_background(void)
{
}
