#include "distance_sensor.h"
#include "robot_config.h"
#include "usart1_log.h"
#include "main.h"
#include <stdlib.h>

#define ROBOT_TYPE 1 //0 is black fully while 1 is black with yellow inside

extern ADC_HandleTypeDef hadc1;

opponent_status_t last_status;

// Global variables for STM32CubeIDE Live Expressions debugging
volatile uint16_t debug_left_adc = 0;
volatile uint16_t debug_front_adc = 0;
volatile uint16_t debug_right_adc = 0;
    
// Define ADC ranges for Sharp IR
// A typical Sharp IR outputs higher voltage at closer distances.

#if ROBOT_TYPE
#define SHARP_IR_SIDE_MIN_ADC_THRESHOLD 500  // Minimum ADC value for Left/Right sensors
#define SHARP_IR_SIDE_MAX_ADC_THRESHOLD 4095  // Maximum ADC value for Left/Right sensors

#define SHARP_IR_FRONT_MIN_ADC_THRESHOLD 500 // Minimum ADC value for Front sensor
#define SHARP_IR_FRONT_MAX_ADC_THRESHOLD 4095 // Maximum ADC value for Front sensor
#else
#define SHARP_IR_SIDE_MIN_ADC_THRESHOLD 300  // Minimum ADC value for Left/Right sensors
#define SHARP_IR_SIDE_MAX_ADC_THRESHOLD 4095  // Maximum ADC value for Left/Right sensors

#define SHARP_IR_FRONT_MIN_ADC_THRESHOLD 500 // Minimum ADC value for Front sensor
#define SHARP_IR_FRONT_MAX_ADC_THRESHOLD 4095 // Maximum ADC value for Front sensor
#endif


static uint16_t read_adc_channel(uint32_t channel)
{
    const uint32_t start_ms = HAL_GetTick();
    
    // Direct register access matching line_sensor.c to prevent HAL lockups
    ADC1->SQR3 = channel;
    
    // Tiny delay to let the analog multiplexer settle physically
    for (volatile int i = 0; i < 50; i++) { __NOP(); }
    
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
        if ((HAL_GetTick() - start_ms) > 2U) {
            return 0U; // Timeout
        }
    }

    return (uint16_t)(ADC1->DR & 0xFFF);
}

static uint16_t filter_adc_spike(uint16_t current, uint16_t *spike_count)
{
    // If the sensor reads a valid object (voltage spike), we require
    // it to be consistent for a few readings before we trust it.
    // Since we are moving fast, we only need 2 consistent readings.
    if (current >= SHARP_IR_SIDE_MIN_ADC_THRESHOLD) {
        (*spike_count)++;
        if (*spike_count > 2) {
            *spike_count = 2; // Cap to prevent overflow
        }
        if (*spike_count >= 2) {
            return current; // Return the high value so it triggers!
        }
    } else {
        *spike_count = 0;
    }
    return 0; // Ignore random spikes
}

static uint8_t is_opponent_detected(uint16_t adc_val, uint16_t min_thresh, uint16_t max_thresh)
{
    if (adc_val >= min_thresh && adc_val <= max_thresh) {
        return 1;
    }
    return 0;
}

void distance_sensor_init(void)
{
    LOG_PRINT("\r\n--- Initializing Sharp IR Distance Sensors ---\r\n");

    // Enable ADC clock and configure sampling times for channels 2, 3, 7 to prevent HAL conflicts
    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC1->CR2 |= ADC_CR2_ADON; // Ensure ADC is powered on
    ADC1->SMPR2 |= ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0 |  // 112 cycles for CH2
                   ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0 |  // 112 cycles for CH3
                   ADC_SMPR2_SMP7_1 | ADC_SMPR2_SMP7_0;   // 112 cycles for CH7

    last_status.front = 0U;
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.rear_right = 0U;
    last_status.rear_left = 0U;
    last_status.distance_mm = 0U;
    
    // Turn off debug LEDs initially
//    HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_RESET);
}

opponent_status_t distance_sensor_read_opponent(void)
{
    // Read the three raw ADC channels
    uint16_t raw_left_adc = read_adc_channel(ADC_CHANNEL_7);
    uint16_t raw_front_adc = read_adc_channel(ADC_CHANNEL_2);
    uint16_t raw_right_adc = read_adc_channel(ADC_CHANNEL_3);
    
    // Static variables to hold the spike count state across loops
    static uint16_t spike_left = 0;
    static uint16_t spike_front = 0;
    static uint16_t spike_right = 0;
    
    // Apply the robust spike filter
    uint16_t left_adc = filter_adc_spike(raw_left_adc, &spike_left);
    uint16_t front_adc = filter_adc_spike(raw_front_adc, &spike_front);
    uint16_t right_adc = filter_adc_spike(raw_right_adc, &spike_right);

    // Save the FILTERED values to globals so they can be viewed in Live Expressions
    debug_left_adc = left_adc;
    debug_front_adc = front_adc;
    debug_right_adc = right_adc;

    last_status.left = is_opponent_detected(left_adc, SHARP_IR_SIDE_MIN_ADC_THRESHOLD, SHARP_IR_SIDE_MAX_ADC_THRESHOLD);
    last_status.front = is_opponent_detected(front_adc, SHARP_IR_FRONT_MIN_ADC_THRESHOLD, SHARP_IR_FRONT_MAX_ADC_THRESHOLD);
    last_status.right = is_opponent_detected(right_adc, SHARP_IR_SIDE_MIN_ADC_THRESHOLD, SHARP_IR_SIDE_MAX_ADC_THRESHOLD);
    
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
//    HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, last_status.left ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, last_status.right ? GPIO_PIN_SET : GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, last_status.front ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return last_status;
}

uint16_t front_mm_return(void)
{
    return last_status.distance_mm;
}
