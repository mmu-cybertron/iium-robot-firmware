#include "line_sensor.h"
#include "main.h"

#define ROBOT_TYPE 0 //1 is black fully while 0 is black with yellow inside

#if ROBOT_TYPE
#define IR_ANALOG_EDGE_THRESHOLD 450U
#define IR_ANALOG_TIMEOUT_MS 1U
#define IR_FL_ADC_CHANNEL 8U
#define IR_FR_ADC_CHANNEL 9U
#define IR_BL_ADC_CHANNEL 6U
#define IR_BR_ADC_CHANNEL 5U
#else
#define IR_ANALOG_EDGE_THRESHOLD 1000U
#define IR_ANALOG_TIMEOUT_MS 1U
#define IR_FL_ADC_CHANNEL 8U
#define IR_FR_ADC_CHANNEL 9U
#define IR_BL_ADC_CHANNEL 6U
#define IR_BR_ADC_CHANNEL 5U
#endif

static uint8_t adc_initialized;

static void line_sensor_adc_init_once(void)
{
    if (adc_initialized != 0U) {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    // GPIO initialization is handled by CubeMX in stm32f4xx_hal_msp.c

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SMPR2 |= ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;
    ADC1->CR2 |= ADC_CR2_ADON;

    adc_initialized = 1U;
}

uint16_t line_sensor_read_adc(uint32_t channel)
{
    const uint32_t start_ms = HAL_GetTick();

    line_sensor_adc_init_once();

    ADC1->SQR3 = channel;
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
        if ((HAL_GetTick() - start_ms) > IR_ANALOG_TIMEOUT_MS) {
            return 4095U;
        }
    }

    return (uint16_t)(ADC1->DR & 0xFFF);
}

void line_sensor_init(void)
{
    line_sensor_adc_init_once();
}

uint16_t debug_edge_left_adc = 0;
uint16_t debug_edge_right_adc = 0;
uint16_t debug_edge_rear_left_adc = 0;
uint16_t debug_edge_rear_right_adc = 0;

edge_status_t line_sensor_read_edges(void)
{
    edge_status_t status;
	debug_edge_left_adc = line_sensor_read_adc(IR_FL_ADC_CHANNEL);
	debug_edge_right_adc = line_sensor_read_adc(IR_FR_ADC_CHANNEL);
    debug_edge_rear_left_adc = line_sensor_read_adc(IR_BL_ADC_CHANNEL);
    debug_edge_rear_right_adc = line_sensor_read_adc(IR_BR_ADC_CHANNEL);



    status.front_left = (debug_edge_left_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.front_right = (debug_edge_right_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.rear_left = (debug_edge_rear_left_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.rear_right = (debug_edge_rear_right_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;

    return status;
}
