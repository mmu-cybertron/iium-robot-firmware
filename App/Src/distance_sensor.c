#include "distance_sensor.h"

#include "main.h"
#include "robot_config.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "vl53l0x_api.h"

extern I2C_HandleTypeDef hi2c1;

static uint8_t is_initialized;
opponent_status_t last_status;

static uint16_t raw_left_mm     = 0U;
static uint16_t raw_front_mm    = 0U;
static uint16_t raw_right_mm    = 0U;
static uint16_t raw_rear_right_mm = 0U;
static uint16_t raw_rear_left_mm  = 0U;

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

    HAL_GPIO_WritePin(XSHUT_4_GPIO_Port, XSHUT_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_5_GPIO_Port, XSHUT_5_Pin, GPIO_PIN_RESET);
    HAL_Delay(20U);

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
    uint8_t status;

    is_initialized = 0U;
    last_status.front = 0U;
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.rear_right      = 0U;   // NEW
    last_status.rear_left = 0U;   // NEW
    last_status.distance_mm = 0U;

    status = VL53L1__InitAll();
    status |= VL53L1X_StartRanging(VL53L1__ADDR_LEFT);
    status |= VL53L1X_StartRanging(VL53L1__ADDR_FRONT);
    status |= VL53L1X_StartRanging(VL53L1__ADDR);
    status |= vl53l0x_init_rear_sensors();

    if (status == 0U) {
        is_initialized = 1U;
    }
}

opponent_status_t distance_sensor_read_opponent(void)
{
    uint16_t rear_right_l1 = 0U;
    uint16_t rear_left_l1  = 0U;

    if (is_initialized == 0U) {
        return last_status;
    }

    (void)VL53L1__ReadAll(&raw_left_mm, &raw_front_mm, &raw_right_mm, &rear_right_l1, &rear_left_l1);
    raw_rear_right_mm = vl53l0x_read_distance(rear_right_handle);
    raw_rear_left_mm  = vl53l0x_read_distance(rear_left_handle);

    last_status.left       = is_valid_target(raw_left_mm);
    last_status.front      = is_valid_target(raw_front_mm);
    last_status.right      = is_valid_target(raw_right_mm);
    last_status.rear_right = is_valid_target(raw_rear_right_mm);
    last_status.rear_left  = is_valid_target(raw_rear_left_mm);
    last_status.distance_mm = nearest_valid_distance(raw_left_mm, raw_front_mm, raw_right_mm, raw_rear_right_mm, raw_rear_left_mm);

    return last_status;
}
