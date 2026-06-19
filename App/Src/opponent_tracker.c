#include "opponent_tracker.h"

#include "distance_sensor.h"
#include "usart1_log.h"

static opponent_status_t current_opponent;

void opponent_tracker_init(void)
{
    distance_sensor_init();
    current_opponent = distance_sensor_read_opponent();
}

void opponent_tracker_update(void)
{
    current_opponent = distance_sensor_read_opponent();
    LOG_PRINT("Front: %u\r\n", current_opponent.front);
    
}

opponent_status_t opponent_tracker_get_status(void)
{
    return current_opponent;
}

uint8_t opponent_tracker_has_target(void)
{
    return (uint8_t)(current_opponent.front ||
                     current_opponent.left ||
                     current_opponent.right ||
					 current_opponent.rear_right ||
					 current_opponent.rear_left);
}
