#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "robot_types.h"

void distance_sensor_init(void);
opponent_status_t distance_sensor_read_opponent(void);
extern opponent_status_t last_status;

uint16_t front_mm_return(void);
void TOF_debug(void);
#endif /* DISTANCE_SENSOR_H */
