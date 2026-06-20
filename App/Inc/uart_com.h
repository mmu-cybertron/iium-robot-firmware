#ifndef VESC_CONTROL_H
#define VESC_CONTROL_H

#include "main.h"            // For STM32 HAL types (UART_HandleTypeDef)
#include "bldc_interface.h"  // For VESC library structures (mc_values)

/* Configuration Parameters */
#define VESC_DEFAULT_BAUDRATE 115200

static void send_packet_to_vesc(unsigned char *data, unsigned int len);
void VESC_UART_init(void);