#include "failsafe.h"

#include "battery_monitor.h"
#include "robot_config.h"

static uint8_t faulted;

void failsafe_init(void)
{
    battery_monitor_init();
    faulted = 0U;
}

void failsafe_update(void)
{
    const uint16_t battery_mv = battery_monitor_read_mv();

    if ((battery_mv > 0U) && (battery_mv < BATTERY_LOW_MV)) {
        faulted = 1U;
    }
}

void failsafe_background(void)
{
}

uint8_t failsafe_is_faulted(void)
{
    return faulted;
}
