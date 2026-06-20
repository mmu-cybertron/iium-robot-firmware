#include "app_main.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot.h"
#include "robot_config.h"
#include "usart1_log.h"
#include "game_mode_selector.h"
#include "motor_control.h"
#include "distance_sensor.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "vesc/vescuart.h"

extern uint32_t HAL_GetTick(void);
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#define SM_Signal_Pin GPIO_PIN_13
#define SM_Signal_GPIO_Port GPIOC

#if ROBOT_ACTIVE_MODE == ROBOT_MODE_LOGGING_ENABLE

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

#endif /* ROBOT_ACTIVE_MODE == ROBOT_MODE_LOGGING_ENABLE */

void app_main(void)
{
    uint32_t last_update_ms = HAL_GetTick();
    uint32_t last_wait_log_ms = HAL_GetTick();
    uint8_t robot_was_running = 0U;

    LOG_PRINT("USART1 logging ready\r\n");

    distance_sensor_init();

    LOG_PRINT("Distance sensors initialized and ranging started\r\n");

    // /* ===== MODE SELECTION LOOP (SM_Signal_Pin == RESET) ===== */
    // LOG_PRINT("Waiting for mode selection (SM_Signal_Pin must be RESET)...\r\n");
    // game_mode_selector_init();

    // while (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) == GPIO_PIN_RESET) {
    //     /* Mode selection update - call regularly for button debouncing */
    //     game_mode_selector_update();

    //     /* Check if mode is locked and ready */
    //     if (game_mode_selector_is_locked()) {
    //         LOG_PRINT("Mode locked. Waiting for SM_Signal_Pin to go HIGH to start game...\r\n");
    //         break;
    //     }
    // }

    // /* ===== GAME LOOP (SM_Signal_Pin == SET) ===== */
    // LOG_PRINT("SM_Signal_Pin is HIGH. Game starting!\r\n");
    // LOG_PRINT("Executing initial move (Mode %d)...\r\n", (int)game_mode_selector_get_mode());

    robot_init();
    LOG_PRINT("Robot initialized, update period: %lu ms\r\n", (unsigned long)ROBOT_UPDATE_PERIOD_MS);

    // /* Execute initial move before state machine starts */
    // while (!game_mode_selector_is_initial_move_done()) {
    //     game_mode_selector_execute_initial_move();
    //     motor_control_update();

    //     const uint32_t now_ms = HAL_GetTick();
    //     if ((now_ms - last_update_ms) >= ROBOT_UPDATE_PERIOD_MS) {
    //         last_update_ms = now_ms;
    //         robot_background();
    //     }
    // }

    // LOG_PRINT("Initial move complete. Entering state machine...\r\n");

    /* Main game loop */
    while (1) {
        const uint32_t now_ms = HAL_GetTick();
        #if ROBOT_ACTIVE_MODE == ROBOT_MODE_LOGGING_ENABLE
            const opponent_status_t tofData = distance_sensor_read_opponent();

            // Log all sensors periodically (every 200ms to avoid flooding UART)
            static uint32_t last_sensor_log_ms = 0U;
            if ((now_ms - last_sensor_log_ms) >= 200U) {
                last_sensor_log_ms = now_ms;
                LOG_PRINT("[TOF] F:%d L:%d R:%d RR:%d RL:%d dist:%umm\r\n",
                        (int)tofData.front,
                        (int)tofData.left,
                        (int)tofData.right,
                        (int)tofData.rear_right,
                        (int)tofData.rear_left,
                        (unsigned int)tofData.distance_mm);
            }
        #endif

        if (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) != GPIO_PIN_SET) {
            if (robot_was_running) {
                robot_was_running = 0U;
                motor_control_stop();
                LOG_PRINT("SM_Signal_Pin LOW. Motors stopped.\r\n");
            }

            if ((now_ms - last_wait_log_ms) >= 1000U) {
                last_wait_log_ms = now_ms;
                LOG_PRINT("Waiting for SM_Signal_Pin HIGH to run robot_update()\r\n");
            }
            robot_background();
            continue;
        }

        robot_was_running = 1U;

        if ((now_ms - last_update_ms) >= ROBOT_UPDATE_PERIOD_MS) {
            last_update_ms = now_ms;
            robot_update();
        }

        robot_background();
    }

    // /* Game ended, reset for next round */
    // LOG_PRINT("SM_Signal_Pin went LOW. Round ended.\r\n");
    // game_mode_selector_reset_for_new_round();
    // motor_control_stop();
    
    // /* Loop back to mode selection for next round */
    // app_main();
}