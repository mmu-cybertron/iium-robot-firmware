#include "app_main.h"

#include <stdint.h>

#include "robot.h"
#include "robot_config.h"

extern uint32_t HAL_GetTick(void);

void app_main(void)
{
    uint32_t last_update_ms = HAL_GetTick();

    robot_init();

    while (1) {
        const uint32_t now_ms = HAL_GetTick();

        if ((now_ms - last_update_ms) >= ROBOT_UPDATE_PERIOD_MS) {
            last_update_ms = now_ms;
            robot_update();
        }

        robot_background();
    }
}
