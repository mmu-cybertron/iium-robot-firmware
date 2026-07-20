#include "line_sensor.h"
#include "main.h"

#define IR_ANALOG_EDGE_THRESHOLD 450U
#define IR_ANALOG_TIMEOUT_MS 1U
#define IR3_ADC_CHANNEL 9U
#define IR4_ADC_CHANNEL 8U
#define REAR_LEFT_ADC_CHANNEL 6U
#define REAR_RIGHT_ADC_CHANNEL 4U

static uint8_t adc_initialized;

static void line_sensor_adc_init_once(void)
{
    if (adc_initialized != 0U) {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PB0, PB1
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SMPR2 |= ADC_SMPR2_SMP4 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;
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

static uint16_t left_adc;
static uint16_t right_adc;
static uint16_t rear_left_adc;
static uint16_t rear_right_adc;

edge_status_t line_sensor_read_edges(void)
{
    edge_status_t status;
	left_adc = line_sensor_read_adc(IR3_ADC_CHANNEL);
	right_adc = line_sensor_read_adc(IR4_ADC_CHANNEL);
    rear_left_adc = line_sensor_read_adc(REAR_LEFT_ADC_CHANNEL);
    rear_right_adc = line_sensor_read_adc(REAR_RIGHT_ADC_CHANNEL);



    status.front_left = (left_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.front_right = (right_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.rear_left = (rear_left_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.rear_right = (rear_right_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;

    return status;
}
