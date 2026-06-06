#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>
#include "robot_types.h"

void line_sensor_init(void);
edge_status_t line_sensor_read_edges(void);

#endif /* LINE_SENSOR_H */
