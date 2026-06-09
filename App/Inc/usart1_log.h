#ifndef USART1_LOG_H
#define USART1_LOG_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef USART1_LOG_TIMEOUT_MS
#define USART1_LOG_TIMEOUT_MS 100U
#endif

#ifndef USART1_LOG_PRINTF_BUFFER_SIZE
#define USART1_LOG_PRINTF_BUFFER_SIZE 128U
#endif

HAL_StatusTypeDef usart1_log_write(const uint8_t *data, size_t length);
HAL_StatusTypeDef usart1_log_write_string(const char *text);
HAL_StatusTypeDef usart1_log_write_line(const char *text);
int usart1_log_printf(const char *format, ...);

#define LOG_PRINT(...) usart1_log_printf(__VA_ARGS__)
#define LOG_LINE(text) usart1_log_write_line(text)

#ifdef __cplusplus
}
#endif

#endif /* USART1_LOG_H */
