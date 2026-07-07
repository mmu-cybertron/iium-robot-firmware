#include "line_sensor.h"
#include "main.h"

#define IR_ANALOG_EDGE_THRESHOLD 450U
#define IR_ANALOG_TIMEOUT_MS 1U
#define IR3_ADC_CHANNEL 9U
#define IR4_ADC_CHANNEL 8U
#define REAR_LEFT_ADC_CHANNEL 6U
#define REAR_RIGHT_ADC_CHANNEL 4U

static uint8_t adc_initialized;
static volatile uint16_t adc_buffer[4] = {4095U, 4095U, 4095U, 4095U};

static void line_sensor_adc_init_once(void)
{
    if (adc_initialized != 0U) {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PB0, PB1
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    // Disable ADC
    ADC1->CR2 &= ~ADC_CR2_ADON;

    // Configure DMA2 Stream0 Channel 0 for ADC1
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN); // wait for disable

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;
    DMA2_Stream0->NDTR = 4;

    // CHSEL = 0, MSIZE = 1 (16-bit), PSIZE = 1 (16-bit), MINC = 1, CIRC = 1, DIR = 0 (P2M)
    DMA2_Stream0->CR = (0U << DMA_SxCR_CHSEL_Pos) |
                       (1U << DMA_SxCR_MSIZE_Pos) |
                       (1U << DMA_SxCR_PSIZE_Pos) |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CIRC |
                       (0U << DMA_SxCR_DIR_Pos);

    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // ADC Config: Scan mode, Continuous conversion, DMA continuous requests
    ADC1->CR1 = ADC_CR1_SCAN; 
    ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    
    // Set sequence: CH4, CH6, CH8, CH9
    ADC1->SQR1 = (3U << 20); // L = 3 (4 conversions)
    ADC1->SQR3 = (4U << 0) | (6U << 5) | (8U << 10) | (9U << 15);
    
    ADC1->SMPR2 |= ADC_SMPR2_SMP4 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;

    // Enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // Start continuous conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    adc_initialized = 1U;
}

uint16_t line_sensor_read_adc(uint32_t channel)
{
    line_sensor_adc_init_once();

    if (channel == REAR_RIGHT_ADC_CHANNEL) return adc_buffer[0]; // CH4
    if (channel == REAR_LEFT_ADC_CHANNEL) return adc_buffer[1];  // CH6
    if (channel == IR4_ADC_CHANNEL) return adc_buffer[2];        // CH8
    if (channel == IR3_ADC_CHANNEL) return adc_buffer[3];        // CH9

    return 4095U;
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

	rear_left_adc = line_sensor_read_adc(4U);
	rear_right_adc = line_sensor_read_adc(6U);

    status.front_left = (left_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.front_right = (right_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.rear_left = (rear_left_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;
    status.rear_right = (rear_right_adc < IR_ANALOG_EDGE_THRESHOLD) ? 1U : 0U;

    return status;
}
