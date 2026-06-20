#include "distance_sensor.h"
#include "robot_config.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "vl53.h"
#include "usart1_log.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
static uint8_t is_initialized;
static opponent_status_t last_status;

static uint8_t is_valid_target(uint16_t distance_mm)
{
    return (uint8_t)((distance_mm > 0U) &&
                     (distance_mm <= OPPONENT_DETECT_DISTANCE_MM));
}

static uint16_t nearest_valid_distance(uint16_t left_mm,
                                       uint16_t front_mm,
                                       uint16_t right_mm,
                                       uint16_t rear_right_mm,
                                       uint16_t rear_left_mm)
{
    uint16_t nearest = 0U;

    if (is_valid_target(front_mm)) {
        nearest = front_mm;
    }
    if (is_valid_target(left_mm) && ((nearest == 0U) || (left_mm < nearest))) {
        nearest = left_mm;
    }
    if (is_valid_target(right_mm) && ((nearest == 0U) || (right_mm < nearest))) {
        nearest = right_mm;
    }
    // ↓ removed duplicate right_mm block that was here
    if (is_valid_target(rear_right_mm) && ((nearest == 0U) || (rear_right_mm < nearest))) {
        nearest = rear_right_mm;
    }
    if (is_valid_target(rear_left_mm) && ((nearest == 0U) || (rear_left_mm < nearest))) {
        nearest = rear_left_mm;
    }

    return nearest;
}

void distance_sensor_init(void)
{
    uint8_t success_count = 0;
    is_initialized = 0U;

    last_status.front = 0U;
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.rear_right = 0U;
    last_status.rear_left = 0U;
    last_status.distance_mm = 0U;

    LOG_PRINT("\r\n--- Initializing Distance Sensors ---\r\n");

    // 1. Initialize VL53L0X (XSHUT4, XSHUT5)
    set_vl53_i2c_handler(&hi2c1);
    if (vl53_init_multi() == HAL_OK) {
        success_count++;
    }

    // 2. Initialize VL53L1X (XSHUT1, XSHUT2, XSHUT3) safely
    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin | XSHUT_2_Pin | XSHUT_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);

    uint8_t err;

    // LEFT
    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    err = VL53L1X_SetI2CAddress(0x52, VL53L1__ADDR_LEFT);
    err |= VL53L1X_SensorInit(VL53L1__ADDR_LEFT);
    if (err == 0) {
        VL53L1X_SetDistanceMode(VL53L1__ADDR_LEFT, VL53L1__DISTANCE_MODE);
        VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR_LEFT, VL53L1__TIMING_BUDGET);
        VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR_LEFT, VL53L1__INTERMEASUREMENT);
        VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
        success_count++;
    }

    // FRONT
    HAL_GPIO_WritePin(GPIOB, XSHUT_2_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    err = VL53L1X_SetI2CAddress(0x52, VL53L1__ADDR_FRONT);
    err |= VL53L1X_SensorInit(VL53L1__ADDR_FRONT);
    if (err == 0) {
        VL53L1X_SetDistanceMode(VL53L1__ADDR_FRONT, VL53L1__DISTANCE_MODE);
        VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR_FRONT, VL53L1__TIMING_BUDGET);
        VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR_FRONT, VL53L1__INTERMEASUREMENT);
        VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
        success_count++;
    }

    // RIGHT
    HAL_GPIO_WritePin(GPIOB, XSHUT_3_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    err = VL53L1X_SensorInit(VL53L1__ADDR);
    if (err == 0) {
        VL53L1X_SetDistanceMode(VL53L1__ADDR, VL53L1__DISTANCE_MODE);
        VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR, VL53L1__TIMING_BUDGET);
        VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR, VL53L1__INTERMEASUREMENT);
        VL53L1X_StartRanging(VL53L1__ADDR);
        success_count++;
    }

    if (success_count > 0) {
        is_initialized = 1U;
        LOG_PRINT("[Sensors] Active and Ready!\r\n");
    }
}

opponent_status_t distance_sensor_read_opponent(void)
{
    uint16_t left_mm = 8191U, front_mm = 8191U, right_mm = 8191U;
    uint16_t rear_right_mm = 8191U, rear_left_mm = 8191U;
    uint16_t dummy = 0U;

    if (is_initialized == 0U) {
        return last_status;
    }

    // 1. Read L1X (Front, Left, Right)
    (void)VL53L1__ReadAll(&left_mm, &front_mm, &right_mm, &dummy, &dummy);

    // 2. Read L0X (Rear Left, Rear Right)
    uint16_t dist_l0x[2] = {8191U, 8191U};
    vl53_read_multi(dist_l0x);
    rear_right_mm = dist_l0x[0]; // Assuming XSHUT4
    rear_left_mm = dist_l0x[1];  // Assuming XSHUT5

    last_status.left = is_valid_target(left_mm);
    last_status.front = is_valid_target(front_mm);
    last_status.right = is_valid_target(right_mm);
    last_status.rear_right = is_valid_target(rear_right_mm);
    last_status.rear_left = is_valid_target(rear_left_mm);
    last_status.distance_mm = nearest_valid_distance(left_mm, front_mm, right_mm, rear_right_mm, rear_left_mm);

    return last_status;
}
