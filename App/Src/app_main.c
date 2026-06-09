#include "app_main.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot.h"
#include "robot_config.h"
#include "usart1_log.h"

extern uint32_t HAL_GetTick(void);
extern UART_HandleTypeDef huart1;

HAL_StatusTypeDef usart1_log_write(const uint8_t *data, size_t length)
{
    if ((data == NULL) && (length > 0U)) {
        return HAL_ERROR;
    }

    while (length > 0U) {
        const uint16_t chunk_length = (length > UINT16_MAX) ? UINT16_MAX : (uint16_t)length;
        const HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1,
                                                           (uint8_t *)data,
                                                           chunk_length,
                                                           USART1_LOG_TIMEOUT_MS);

        if (status != HAL_OK) {
            return status;
        }

        data += chunk_length;
        length -= chunk_length;
    }

    return HAL_OK;
}

HAL_StatusTypeDef usart1_log_write_string(const char *text)
{
    if (text == NULL) {
        return HAL_ERROR;
    }

    return usart1_log_write((const uint8_t *)text, strlen(text));
}

HAL_StatusTypeDef usart1_log_write_line(const char *text)
{
    HAL_StatusTypeDef status = usart1_log_write_string(text);

    if (status != HAL_OK) {
        return status;
    }

    return usart1_log_write((const uint8_t *)"\r\n", 2U);
}

int usart1_log_printf(const char *format, ...)
{
    char buffer[USART1_LOG_PRINTF_BUFFER_SIZE];
    va_list args;

    va_start(args, format);
    const int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0) {
        return length;
    }

    const size_t output_length = ((size_t)length < sizeof(buffer)) ? (size_t)length : (sizeof(buffer) - 1U);

    if (usart1_log_write((const uint8_t *)buffer, output_length) != HAL_OK) {
        return -1;
    }

    return length;
}

int __io_putchar(int ch)
{
    const uint8_t data = (uint8_t)ch;

    if (ch == '\n') {
        const uint8_t carriage_return = '\r';
        (void)usart1_log_write(&carriage_return, 1U);
    }

    (void)usart1_log_write(&data, 1U);
    return ch;
}

int __io_getchar(void)
{
    uint8_t data = 0U;

    (void)HAL_UART_Receive(&huart1, &data, 1U, HAL_MAX_DELAY);
    return (int)data;
}

void app_main(void)
{
    uint32_t last_update_ms = HAL_GetTick();

    robot_init();
    LOG_PRINT("USART1 logging ready, update period: %lu ms\r\n", (unsigned long)ROBOT_UPDATE_PERIOD_MS);

    while (1) {
        const uint32_t now_ms = HAL_GetTick();

        if ((now_ms - last_update_ms) >= ROBOT_UPDATE_PERIOD_MS) {
            last_update_ms = now_ms;
            robot_update();
        }

        robot_background();
    }
}
