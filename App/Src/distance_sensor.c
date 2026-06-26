#include "distance_sensor.h"
#include "robot_config.h"
#include "usart1_log.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "vl53.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define DISTANCE_SENSOR_ENABLE_REAR_VL53L0X 1
#define VL53L1_FAILURE_LEFT                 0x01U
#define VL53L1_FAILURE_FRONT                0x02U
#define VL53L1_FAILURE_RIGHT                0x04U
#define VL53L1_FAILURE_ALL                  (VL53L1_FAILURE_LEFT | \
                                             VL53L1_FAILURE_FRONT | \
                                             VL53L1_FAILURE_RIGHT)
#define VL53L1_FAILURE_RECOVERY_THRESHOLD   3U
#define VL53L1_STALE_HOLD_MS                250U
#define VL53L1_I2C_RECOVERY_CLOCK_PULSES    9U
#define VL53L1_POLL_PERIOD_MS               VL53L1__INTERMEASUREMENT

static uint8_t is_initialized;
opponent_status_t last_status;

typedef enum {
    VL53L1_SENSOR_LEFT = 0,
    VL53L1_SENSOR_FRONT,
    VL53L1_SENSOR_RIGHT,
    VL53L1_SENSOR_COUNT
} vl53l1_sensor_index_t;

typedef struct {
    uint16_t distance_mm;
    uint32_t updated_ms;
    uint8_t has_value;
    uint8_t consecutive_failures;
} vl53l1_sensor_cache_t;

static vl53l1_sensor_cache_t vl53l1_cache[VL53L1_SENSOR_COUNT];
static uint8_t vl53l1_fault_active;
static uint32_t vl53l1_last_poll_ms;

#if DISTANCE_SENSOR_ENABLE_REAR_VL53L0X
static VL53L0X_Dev_t rear_right_device;
static VL53L0X_Dev_t rear_left_device;
static VL53L0X_DEV rear_right_handle = &rear_right_device;
static VL53L0X_DEV rear_left_handle = &rear_left_device;

static uint16_t vl53l0x_read_distance(VL53L0X_DEV dev)
{
    VL53L0X_RangingMeasurementData_t measurement;
    uint8_t ready = 0U;

    if (VL53L0X_GetMeasurementDataReady(dev, &ready) != VL53L0X_ERROR_NONE) {
        return 0U;
    }

    if (ready == 0U) {
        return 0U;
    }

    if (VL53L0X_GetRangingMeasurementData(dev, &measurement) != VL53L0X_ERROR_NONE) {
        return 0U;
    }

    (void)VL53L0X_ClearInterruptMask(dev, 0U);

    if (measurement.RangeStatus != 0U) {
        return 0U;
    }

    return measurement.RangeMilliMeter;
}

static uint8_t vl53l0x_init_rear_sensor(VL53L0X_DEV dev,
                                        GPIO_TypeDef *port,
                                        uint16_t pin,
                                        uint8_t address)
{
    uint8_t status = VL53L0X_ERROR_NONE;
    uint32_t ref_spad_count = 0U;
    uint8_t is_aperture_spads = 0U;
    uint8_t vhv_settings = 0U;
    uint8_t phase_cal = 0U;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_Delay(2U);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(2U);

    dev->I2cHandle = &hi2c1;
    dev->I2cDevAddr = 0x52U;

    status = VL53L0X_WaitDeviceBooted(dev);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_DataInit(dev);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_StaticInit(dev);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_PerformRefCalibration(dev, &vhv_settings, &phase_cal);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_PerformRefSpadManagement(dev, &ref_spad_count, &is_aperture_spads);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetLimitCheckValue(dev,
                                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                        (FixPoint1616_t)(0.1F * 65536.0F));
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetLimitCheckValue(dev,
                                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        (FixPoint1616_t)(60.0F * 65536.0F));
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 20000U);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetVcselPulsePeriod(dev,
                                         VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                         18U);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetVcselPulsePeriod(dev,
                                         VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                         14U);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetDeviceAddress(dev, address);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    dev->I2cDevAddr = address;

    return VL53L0X_StartMeasurement(dev);
}

static uint8_t vl53l0x_init_rear_sensors(void)
{
    uint8_t status = VL53L0X_ERROR_NONE;

    status |= vl53l0x_init_rear_sensor(rear_right_handle,
                                        XSHUT_4_GPIO_Port,
                                        XSHUT_4_Pin,
                                        0x58U);
    status |= vl53l0x_init_rear_sensor(rear_left_handle,
                                        XSHUT_5_GPIO_Port,
                                        XSHUT_5_Pin,
                                        0x5AU);

    return status;
}
#endif

static uint8_t is_valid_target(uint16_t distance_mm)
{
    return (uint8_t)((distance_mm > 0U) &&
                     (distance_mm <= OPPONENT_DETECT_DISTANCE_MM));
}

static void distance_sensor_update_debug_leds(const opponent_status_t *status)
{
    HAL_GPIO_WritePin(LED_D6_GPIO_Port,
                      LED_D6_Pin,
                      status->front ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D8_GPIO_Port,
                      LED_D8_Pin,
                      vl53l1_fault_active ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void distance_sensor_signal_recovery(void)
{
    HAL_GPIO_TogglePin(LED_D7_GPIO_Port, LED_D7_Pin);
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

static uint8_t distance_sensor_start_vl53l1(void)
{
    uint8_t success_count = 0U;

    last_status.front = 0U;
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.rear_right = 0U;
    last_status.rear_left = 0U;
    last_status.distance_mm = 0U;
    distance_sensor_update_debug_leds(&last_status);

    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin | XSHUT_2_Pin | XSHUT_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(20U);

    uint8_t err;

    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin, GPIO_PIN_SET);
    HAL_Delay(20U);
    err = VL53L1X_SetI2CAddress(0x52, VL53L1__ADDR_LEFT);
    err |= VL53L1X_SensorInit(VL53L1__ADDR_LEFT);
    if (err == 0U) {
        err |= VL53L1X_SetDistanceMode(VL53L1__ADDR_LEFT, VL53L1__DISTANCE_MODE);
        err |= VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR_LEFT, VL53L1__TIMING_BUDGET);
        err |= VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR_LEFT, VL53L1__INTERMEASUREMENT);
        err |= VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
        if (err == 0U) {
            success_count++;
        }
    }

    HAL_GPIO_WritePin(GPIOB, XSHUT_2_Pin, GPIO_PIN_SET);
    HAL_Delay(20U);
    err = VL53L1X_SetI2CAddress(0x52, VL53L1__ADDR_FRONT);
    err |= VL53L1X_SensorInit(VL53L1__ADDR_FRONT);
    if (err == 0U) {
        err |= VL53L1X_SetDistanceMode(VL53L1__ADDR_FRONT, VL53L1__DISTANCE_MODE);
        err |= VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR_FRONT, VL53L1__TIMING_BUDGET);
        err |= VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR_FRONT, VL53L1__INTERMEASUREMENT);
        err |= VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
        if (err == 0U) {
            success_count++;
        }
    }

    HAL_GPIO_WritePin(GPIOB, XSHUT_3_Pin, GPIO_PIN_SET);
    HAL_Delay(20U);
    err = VL53L1X_SensorInit(VL53L1__ADDR);
    if (err == 0U) {
        err |= VL53L1X_SetDistanceMode(VL53L1__ADDR, VL53L1__DISTANCE_MODE);
        err |= VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR, VL53L1__TIMING_BUDGET);
        err |= VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR, VL53L1__INTERMEASUREMENT);
        err |= VL53L1X_StartRanging(VL53L1__ADDR);
        if (err == 0U) {
            success_count++;
        }
    }

    return success_count;
}

static void distance_sensor_clear_cache(void)
{
    for (uint8_t i = 0U; i < (uint8_t)VL53L1_SENSOR_COUNT; i++) {
        vl53l1_cache[i].distance_mm = 0U;
        vl53l1_cache[i].updated_ms = 0U;
        vl53l1_cache[i].has_value = 0U;
        vl53l1_cache[i].consecutive_failures = 0U;
    }

    vl53l1_last_poll_ms = 0U;
}

static uint8_t distance_sensor_has_all_cached_readings(void)
{
    return (uint8_t)((vl53l1_cache[VL53L1_SENSOR_LEFT].has_value != 0U) &&
                     (vl53l1_cache[VL53L1_SENSOR_FRONT].has_value != 0U) &&
                     (vl53l1_cache[VL53L1_SENSOR_RIGHT].has_value != 0U));
}

static void distance_sensor_configure_i2c_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
}

static void distance_sensor_pulse_i2c_clock(void)
{
    distance_sensor_configure_i2c_gpio();

    for (uint8_t i = 0U; i < VL53L1_I2C_RECOVERY_CLOCK_PULSES; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1U);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

static void distance_sensor_recover_i2c_bus(void)
{
    (void)HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(1U);
    __HAL_RCC_I2C1_RELEASE_RESET();
    distance_sensor_pulse_i2c_clock();
    (void)HAL_I2C_Init(&hi2c1);

    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET) {
        (void)HAL_I2C_DeInit(&hi2c1);
        distance_sensor_pulse_i2c_clock();
        (void)HAL_I2C_Init(&hi2c1);
    }
}

static void distance_sensor_recover_vl53l1(uint8_t recover_i2c)
{
    vl53l1_fault_active = 1U;
    distance_sensor_update_debug_leds(&last_status);
    distance_sensor_signal_recovery();

    if (recover_i2c != 0U) {
        distance_sensor_recover_i2c_bus();
    }

    is_initialized = (distance_sensor_start_vl53l1() > 0U) ? 1U : 0U;
    distance_sensor_clear_cache();
}

static uint8_t distance_sensor_use_reading(vl53l1_sensor_index_t sensor,
                                           uint8_t failure_mask,
                                           uint8_t sensor_failure_bit,
                                           uint16_t raw_distance_mm,
                                           uint32_t now_ms,
                                           uint16_t *distance_mm)
{
    vl53l1_sensor_cache_t *cache = &vl53l1_cache[sensor];

    if ((failure_mask & sensor_failure_bit) == 0U) {
        cache->distance_mm = raw_distance_mm;
        cache->updated_ms = now_ms;
        cache->has_value = 1U;
        cache->consecutive_failures = 0U;
        *distance_mm = raw_distance_mm;
        return 0U;
    }

    if (cache->consecutive_failures < UINT8_MAX) {
        cache->consecutive_failures++;
    }

    if ((cache->has_value != 0U) && ((now_ms - cache->updated_ms) <= VL53L1_STALE_HOLD_MS)) {
        *distance_mm = cache->distance_mm;
    } else {
        *distance_mm = 0U;
    }

    return 1U;
}

void distance_sensor_init(void)
{
//    is_initialized = 0U;
//    vl53l1_fault_active = 0U;
//
//    last_status.front = 0U;
//    last_status.left = 0U;
//    last_status.right = 0U;
//    last_status.rear_right = 0U;
//    last_status.rear_left = 0U;
//    last_status.distance_mm = 0U;
//    distance_sensor_clear_cache();
//    distance_sensor_update_debug_leds(&last_status);
//
//#if DISTANCE_SENSOR_ENABLE_REAR_VL53L0X
//    if (vl53l0x_init_rear_sensors() != VL53L0X_ERROR_NONE) {
//        vl53l1_fault_active = 1U;
//        distance_sensor_update_debug_leds(&last_status);
//    }
//#endif
//
//    if (distance_sensor_start_vl53l1() > 0U) {
//        is_initialized = 1U;
//    } else {
//        vl53l1_fault_active = 1U;
//        distance_sensor_update_debug_leds(&last_status);
//    }
//
//    vl53l1_last_poll_ms = 0U;

	is_initialized = 0U;
	    vl53l1_fault_active = 0U;
	    distance_sensor_clear_cache();

	    // 1. Force ALL XSHUT lines LOW to turn off every single sensor on the robot
	    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin | XSHUT_2_Pin | XSHUT_3_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(XSHUT_4_GPIO_Port, XSHUT_4_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(XSHUT_5_GPIO_Port, XSHUT_5_Pin, GPIO_PIN_RESET);
	    HAL_Delay(50U); // Let them fully power down

	    // 2. Initialize the front VL53L1X array first (this steps through pins 1, 2, and 3 internally)
	    if (distance_sensor_start_vl53l1() > 0U) {
	        is_initialized = 1U;
	    } else {
	        vl53l1_fault_active = 1U;
	    }

	#if DISTANCE_SENSOR_ENABLE_REAR_VL53L0X
	    // 3. Now that the front sensors have custom addresses assigned, initialize the rear ones
	    // Remove the reset lines inside vl53l0x_init_rear_sensor so they don't break the bus!
	    if (vl53l0x_init_rear_sensors() != VL53L0X_ERROR_NONE) {
	        vl53l1_fault_active = 1U;
	    }
	#endif

	    distance_sensor_update_debug_leds(&last_status);
	    vl53l1_last_poll_ms = 0U;
}

opponent_status_t distance_sensor_read_opponent(void)
{
    uint16_t raw_left_mm = 0U, raw_front_mm = 0U, raw_right_mm = 0U;
    uint16_t left_mm = 0U, front_mm = 0U, right_mm = 0U;
    uint16_t rear_right_mm = 8191U, rear_left_mm = 8191U;
    uint16_t dummy = 0U;
    uint8_t failure_mask;
    uint8_t read_failed = 0U;
    const uint32_t now_ms = HAL_GetTick();

    if (is_initialized == 0U) {
        distance_sensor_recover_vl53l1(1U);
        distance_sensor_update_debug_leds(&last_status);
        return last_status;
    }

    if (((now_ms - vl53l1_last_poll_ms) < VL53L1_POLL_PERIOD_MS) &&
        (distance_sensor_has_all_cached_readings() != 0U)) {
        left_mm = vl53l1_cache[VL53L1_SENSOR_LEFT].distance_mm;
        front_mm = vl53l1_cache[VL53L1_SENSOR_FRONT].distance_mm;
        right_mm = vl53l1_cache[VL53L1_SENSOR_RIGHT].distance_mm;
    } else {
        vl53l1_last_poll_ms = now_ms;
        failure_mask = VL53L1__ReadAll(&raw_left_mm, &raw_front_mm, &raw_right_mm, &dummy, &dummy);
//        read_failed |= distance_sensor_use_reading(VL53L1_SENSOR_LEFT,
//                                                   failure_mask,
//                                                   VL53L1_FAILURE_LEFT,
//                                                   raw_left_mm,
//                                                   now_ms,
//                                                   &left_mm);
//        read_failed |= distance_sensor_use_reading(VL53L1_SENSOR_FRONT,
//                                                   failure_mask,
//                                                   VL53L1_FAILURE_FRONT,
//                                                   raw_front_mm,
//                                                   now_ms,
//                                                   &front_mm);
//        read_failed |= distance_sensor_use_reading(VL53L1_SENSOR_RIGHT,
//                                                   failure_mask,
//                                                   VL53L1_FAILURE_RIGHT,
//                                                   raw_right_mm,
//                                                   now_ms,
//                                                   &right_mm);
//
//        if ((failure_mask & VL53L1_FAILURE_ALL) == VL53L1_FAILURE_ALL) {
//            distance_sensor_recover_vl53l1(1U);
//        } else if ((vl53l1_cache[VL53L1_SENSOR_LEFT].consecutive_failures >= VL53L1_FAILURE_RECOVERY_THRESHOLD) ||
//                   (vl53l1_cache[VL53L1_SENSOR_FRONT].consecutive_failures >= VL53L1_FAILURE_RECOVERY_THRESHOLD) ||
//                   (vl53l1_cache[VL53L1_SENSOR_RIGHT].consecutive_failures >= VL53L1_FAILURE_RECOVERY_THRESHOLD)) {
//            distance_sensor_recover_vl53l1(0U);
//        }
    }

#if DISTANCE_SENSOR_ENABLE_REAR_VL53L0X
    rear_right_mm = vl53l0x_read_distance(rear_right_handle);
    rear_left_mm = vl53l0x_read_distance(rear_left_handle);
#else
    rear_right_mm = 0U;
    rear_left_mm = 0U;
#endif

    last_status.left = is_valid_target(left_mm);
    last_status.front = is_valid_target(front_mm);
    last_status.right = is_valid_target(right_mm);
    last_status.rear_right = is_valid_target(rear_right_mm);
    last_status.rear_left = is_valid_target(rear_left_mm);
    last_status.distance_mm = nearest_valid_distance(left_mm, front_mm, right_mm, rear_right_mm, rear_left_mm);

    if (read_failed == 0U) {
        vl53l1_fault_active = 0U;
    }
    distance_sensor_update_debug_leds(&last_status);

    return last_status;
}

void TOF_debug(void)
{
	if ((last_status.front != 0U) && (last_status.distance_mm > 0U)) {
		LOG_PRINT("%u front,\r\n", last_status.distance_mm);
	} else if ((last_status.left != 0U) && (last_status.distance_mm > 0U)) {
    	LOG_PRINT("%u left,\r\n", last_status.distance_mm);
	} else if ((last_status.right != 0U) && (last_status.distance_mm > 0U)) {
    	LOG_PRINT("%u right,\r\n", last_status.distance_mm);
	} else if ((last_status.rear_left != 0U) && (last_status.distance_mm > 0U)) {
    	LOG_PRINT("%u rear left,\r\n", last_status.distance_mm);
	} else if ((last_status.rear_right != 0U) && (last_status.distance_mm > 0U)) {
    	LOG_PRINT("%u rear right,\r\n", last_status.distance_mm);
	} else {
		LOG_PRINT("OUT OF SCOPE\r\n");
	}
}

uint16_t front_mm_return(void)
{
    (void)distance_sensor_read_opponent();

    if ((last_status.front != 0U) && (last_status.distance_mm > 0U)) {
        return last_status.distance_mm;
    }

    return 0U;
}
