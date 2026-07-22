#include "distance_sensor.h"
#include "robot_config.h"
#include "usart1_log.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

opponent_status_t last_status;

// Global variables for STM32CubeIDE Live Expressions debugging
volatile uint16_t debug_left_adc = 0;
volatile uint16_t debug_front_adc = 0;
volatile uint16_t debug_right_adc = 0;
// Define ADC ranges for Sharp IR
// A typical Sharp IR outputs higher voltage at closer distances.
// The user requested a range rather than a single threshold.
#define SHARP_IR_MIN_ADC_THRESHOLD 1500  // Minimum ADC value to be considered an opponent
#define SHARP_IR_MAX_ADC_THRESHOLD 4000  // Maximum ADC value (to filter out noise/glitches)

static uint16_t read_adc_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0;
    }

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return 0;
}

static uint8_t is_opponent_detected(uint16_t adc_val)
{
    if (adc_val >= SHARP_IR_MIN_ADC_THRESHOLD && adc_val <= SHARP_IR_MAX_ADC_THRESHOLD) {
        return 1;
    }
    return 0;
}

void distance_sensor_init(void)
{
    LOG_PRINT("\r\n--- Initializing Sharp IR Distance Sensors ---\r\n");

    last_status.front = 0U;
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.rear_right = 0U;
    last_status.rear_left = 0U;
    last_status.distance_mm = 0U;
    
    // Turn off debug LEDs initially
    HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_RESET);
}

opponent_status_t distance_sensor_read_opponent(void)
{
    // Read the three ADC channels
    // PA7 -> Left -> ADC1_IN7
    // PA2 -> Front -> ADC1_IN2
    // PA3 -> Right -> ADC1_IN3

    uint16_t left_adc = read_adc_channel(ADC_CHANNEL_7);
    uint16_t front_adc = read_adc_channel(ADC_CHANNEL_2);
    uint16_t right_adc = read_adc_channel(ADC_CHANNEL_3);

    // Save to globals so they can be viewed in Live Expressions
    debug_left_adc = left_adc;
    debug_front_adc = front_adc;
    debug_right_adc = right_adc;

    last_status.left = is_opponent_detected(left_adc);
    last_status.front = is_opponent_detected(front_adc);
    last_status.right = is_opponent_detected(right_adc);
    
    // Rear sensors not implemented with Sharp IRs in this setup
    last_status.rear_right = 0U;
    last_status.rear_left = 0U;

    // Set distance_mm based on whether any sensor detects an opponent.
    if (last_status.left || last_status.front || last_status.right) {
        last_status.distance_mm = 200; // Arbitrary valid distance > 0
    } else {
        last_status.distance_mm = 0;
    }

    // Update debug LEDs based on detection
    HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, last_status.left ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, last_status.right ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, last_status.front ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return last_status;
}

uint16_t front_mm_return(void)
{
    return last_status.distance_mm;
}
