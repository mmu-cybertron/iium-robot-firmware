#include "robot.h"
#include "robot_config.h"
#include "main.h"

#include "motion.h"
#include "edge_detector.h"
#include "failsafe.h"
#include "motor_control.h"
#include "opponent_tracker.h"
#include "state_machine.h"
#include "usart1_log.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"

#define VL53_TEST_XSHUT_GPIO_Port GPIOB
#define VL53_TEST_XSHUT_Pin XSHUT_1_Pin
#define VL53_TEST_LOG_PERIOD_MS 100U

static uint8_t vl53_test_initialized;

static void vl53_test_disable_all_xshut(void)
{
    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, XSHUT_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, XSHUT_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, XSHUT_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, XSHUT_5_Pin, GPIO_PIN_RESET);
}

static uint8_t vl53_test_wait_boot(uint32_t timeout_ms)
{
    uint8_t boot_state = 0U;
    const uint32_t start_ms = HAL_GetTick();

    while ((boot_state == 0U) && ((HAL_GetTick() - start_ms) < timeout_ms)) {
        if (VL53L1X_BootState(VL53L1__ADDR, &boot_state) != 0) {
            return 1U;
        }
        HAL_Delay(2);
    }

    return (uint8_t)(boot_state == 0U);
}

static void vl53_test_init(void)
{
    uint8_t status = 0U;

    vl53_test_disable_all_xshut();
    HAL_Delay(10);

    HAL_GPIO_WritePin(VL53_TEST_XSHUT_GPIO_Port, VL53_TEST_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    status |= vl53_test_wait_boot(100U);
    status |= VL53L1X_SensorInit(VL53L1__ADDR);
    status |= VL53L1X_SetDistanceMode(VL53L1__ADDR, VL53L1__DISTANCE_MODE);
    status |= VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR, VL53L1__TIMING_BUDGET);
    status |= VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR, VL53L1__INTERMEASUREMENT);
    status |= VL53L1X_StartRanging(VL53L1__ADDR);

    vl53_test_initialized = (uint8_t)(status == 0U);

    if (vl53_test_initialized) {
        LOG_PRINT("VL53L1X single sensor ready at 0x%02X\r\n", VL53L1__ADDR);
    } else {
        LOG_PRINT("VL53L1X init failed, status=%u\r\n", status);
    }
}

static void vl53_test_update(void)
{
    static uint32_t last_log_ms;
    const uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_log_ms) < VL53_TEST_LOG_PERIOD_MS) {
        return;
    }
    last_log_ms = now_ms;

    if (!vl53_test_initialized) {
        LOG_PRINT("VL53L1X not initialized\r\n");
        return;
    }

    const uint16_t distance_mm = VL53L1__ReadOne(VL53L1__ADDR);

    if (distance_mm > 0U) {
        LOG_PRINT("VL53L1X distance: %u mm\r\n", distance_mm);
    } else {
        LOG_PRINT("VL53L1X distance read timeout/status error\r\n");
    }
}

void robot_init(void)
{
    LOG_PRINT("VL53L1X single sensor test starting\r\n");
    vl53_test_init();
}

// Where the robot reads sensors, decides behavior, and updates motor PWM.
// Examples: read sensors, update edge detection, update opponent detection, choose movement, update motors

void robot_update(void)
{
    vl53_test_update();
}

// Call as often as possible inside the infinite loop. It is for non-timing-critical background tasks.
// Examples: checking communication, debug LED blinking, low-priority monitoring, background failsafe work, telemetry later

void robot_background(void)
{
}


