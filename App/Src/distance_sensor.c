#include "distance_sensor.h"
#include "robot_config.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "vl53.h"
#include "usart1_log.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define DISTANCE_SENSOR_ENABLE_REAR_VL53L0X 0

static uint8_t is_initialized;
opponent_status_t last_status;

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

// static void distance_sensor_update_debug_leds(const opponent_status_t *status)
// {
//     HAL_GPIO_WritePin(LED_D6_GPIO_Port,
//                       LED_D6_Pin,
//                       status->left ? GPIO_PIN_SET : GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(LED_D7_GPIO_Port,
//                       LED_D7_Pin,
//                       status->front ? GPIO_PIN_SET : GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(LED_D8_GPIO_Port,
//                       LED_D8_Pin,
//                       status->right ? GPIO_PIN_SET : GPIO_PIN_RESET);
// }

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
    //distance_sensor_update_debug_leds(&last_status);

    status = VL53L1__InitAll();
    status |= VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
    status |= VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
    status |= VL53L1X_StartRanging(VL53L1__ADDR);
#if DISTANCE_SENSOR_ENABLE_REAR_VL53L0X
    status |= vl53l0x_init_rear_sensors();
#endif
    /* Rear VL53L0X sensors are disabled for the current 3x VL53L1X setup. */

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
        //distance_sensor_update_debug_leds(&last_status);
        return last_status;
    }

    (void)VL53L1__ReadAll(&left_mm, &front_mm, &right_mm, &rear_right_l1, &rear_left_l1);
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
    //distance_sensor_update_debug_leds(&last_status);

    //testing code
    if (last_status.front) {
        HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_SET);
    }
    


    return last_status;
}
