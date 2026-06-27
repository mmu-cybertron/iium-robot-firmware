#include "line_sensor.h"
#include "main.h"

#define IR_ANALOG_EDGE_THRESHOLD 2600U
#define IR_ANALOG_TIMEOUT_MS 1U
#define IR3_ADC_CHANNEL 9U
#define IR4_ADC_CHANNEL 8U

static uint8_t adc_initialized;

static void line_sensor_adc_init_once(void)
{
    if (adc_initialized != 0U) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    gpio.Pin = IR3_DO_Pin | IR4_DO_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SMPR2 |= ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;
    ADC1->CR2 |= ADC_CR2_ADON;

    adc_initialized = 1U;
}

static uint16_t line_sensor_read_adc(uint32_t channel)
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

    return (uint16_t)ADC1->DR;
}

void line_sensor_init(void)
{
    line_sensor_adc_init_once();
}

edge_status_t line_sensor_read_edges(void)
{
    edge_status_t status;
    const uint16_t left_adc = line_sensor_read_adc(IR3_ADC_CHANNEL);
    const uint16_t right_adc = line_sensor_read_adc(IR4_ADC_CHANNEL);

    status.front_left = (left_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.front_right = (right_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.rear_left = 0U;
    status.rear_right = 0U;

    return status;
}
