#include "distance_sensor.h"

#include "robot_config.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"

static uint8_t is_initialized;
static opponent_status_t last_status;

static uint8_t is_valid_target(uint16_t distance_mm)
{
    return (uint8_t)((distance_mm > 0U) &&
                     (distance_mm <= OPPONENT_DETECT_DISTANCE_MM));
}

static uint16_t nearest_valid_distance(uint16_t left_mm,
                                       uint16_t front_mm,
                                       uint16_t right_mm)
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

    return nearest;
}

void distance_sensor_init(void)
{
    uint8_t status;

    is_initialized = 0U;
    last_status.front = 0U;
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.distance_mm = 0U;

    status = VL53L1__InitAll();
    status |= VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
    status |= VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
    status |= VL53L1X_StartRanging(VL53L1__ADDR);

    if (status == 0U) {
        is_initialized = 1U;
    }
}

opponent_status_t distance_sensor_read_opponent(void)
{
    uint16_t left_mm = 0U;
    uint16_t front_mm = 0U;
    uint16_t right_mm = 0U;

    if (is_initialized == 0U) {
        return last_status;
    }

    (void)VL53L1__ReadAll(&left_mm, &front_mm, &right_mm);

    last_status.left = is_valid_target(left_mm);
    last_status.front = is_valid_target(front_mm);
    last_status.right = is_valid_target(right_mm);
    last_status.distance_mm = nearest_valid_distance(left_mm, front_mm, right_mm);

    return last_status;
}
