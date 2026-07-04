#include "distance_sensor.h"
#include "robot_config.h"
#include "usart1_log.h"
#include "main.h"
#include "line_sensor.h"


opponent_status_t last_status;
uint32_t debug_sharp_adc_val = 0; // GLOBAL FOR LIVE EXPRESSIONS

static void distance_sensor_update_debug_leds(const opponent_status_t *status)
{
    HAL_GPIO_WritePin(LED_D6_GPIO_Port,
                      LED_D6_Pin,
                      status->left ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D7_GPIO_Port,
                      LED_D7_Pin,
                      status->right ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D8_GPIO_Port,
                      LED_D8_Pin,
                      status->front ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void distance_sensor_init(void)
{
    last_status.front = 0U;
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.rear_right = 0U;
    last_status.rear_left = 0U;
    last_status.distance_mm = 0U;
    
    distance_sensor_update_debug_leds(&last_status);

#if ROBOT_ENABLE_SHARP_IR_SENSOR
    LOG_PRINT("\r\n--- Initializing Sharp IR Distance Sensor ---\r\n");
    LOG_PRINT("[Sensors] Active and Ready!\r\n");
#endif
}

uint8_t distance_sensor_start_vl53l1(void)
{
    return 1U;
}

uint8_t distance_sensor_needs_recovery(void)
{
    return 0U;
}

void distance_sensor_recover_during_edge_escape(void)
{
}

opponent_status_t distance_sensor_read_opponent(void)
{
#if ROBOT_ENABLE_SHARP_IR_SENSOR
    uint32_t adc_val = line_sensor_read_adc(SHARP_IR_ADC_CHANNEL);
    debug_sharp_adc_val = adc_val; // Save to global for Live Expressions
    static uint32_t last_log_time = 0;
    uint32_t now = HAL_GetTick();
    
    if (now - last_log_time >= 500)
    {
        LOG_PRINT("ADC Raw: %lu | Threshold: %u\r\n", (unsigned long)adc_val, SHARP_IR_THRESHOLD);
        last_log_time = now;
    }
    
    if (adc_val > SHARP_IR_THRESHOLD)
    {
        last_status.front = 1U;
    }
    else
    {
        last_status.front = 0U;
    }
#else
    last_status.front = 0U;
#endif
    
    last_status.left = 0U;
    last_status.right = 0U;
    last_status.rear_right = 0U;
    last_status.rear_left = 0U;
    last_status.distance_mm = 0U;
    
    distance_sensor_update_debug_leds(&last_status);
    
    return last_status;
}

uint16_t front_mm_return(void)
{
    if (last_status.front != 0U) {
        return 0U;
    }
    return 8191U;
}

void TOF_debug(void)
{
    LOG_PRINT("IR Front: %d\r\n", last_status.front);
}
