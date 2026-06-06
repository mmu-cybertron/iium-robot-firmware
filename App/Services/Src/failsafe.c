#include "failsafe.h"

#include "main.h"
#include "robot_config.h"

#define START_MODULE_ACTIVE_STATE GPIO_PIN_SET

static uint8_t faulted;
static uint8_t match_started;
static uint32_t start_time_ms;

void failsafe_init(void)
{
    faulted = 1U;
    match_started = 0U;
    start_time_ms = 0U;
}

void failsafe_update(void)
{
    GPIO_PinState signal = HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin);

    if (signal != START_MODULE_ACTIVE_STATE) {
        match_started = 0U;
        faulted = 1U;
        return;
    }

    if (match_started == 0U) {
        match_started = 1U;
        start_time_ms = HAL_GetTick();
        faulted = 1U;
        return;
    }

    faulted = 0U;
}

void failsafe_background(void)
{
}

uint8_t failsafe_is_faulted(void)
{
    return faulted;
}