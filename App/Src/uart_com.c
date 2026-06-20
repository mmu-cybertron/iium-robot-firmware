#include "uart_com.h"
#include "bldc_interface.h"
#include "bldc_interface_uart.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

static uint8_t v_rx_byte;

static void send_packet_to_vesc(unsigned char *data, unsigned int len) 
{
    // Pass the library bytes directly out of USART2 (PA3 Pin)
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
}

void VESC_UART_init(void)
{
    bldc_interface_uart_init(send_packet_to_vesc);
    
    HAL_UART_Receive_IT(&huart2, &v_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    if (huart->Instance == USART2) 
    {
        // Pass the fresh byte directly into Benjamin's packet assembler state machine
        bldc_interface_uart_process_byte(v_rx_byte);
        
        // Re-prime the interrupt so the hardware listens for the next byte
        HAL_UART_Receive_IT(&huart2, &v_rx_byte, 1);
    }
}