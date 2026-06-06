#ifndef EDGE_DETECTOR_H
#define EDGE_DETECTOR_H

#include "robot_types.h"

void edge_detector_init(void);
void edge_detector_update(void);
edge_status_t edge_detector_get_status(void);
uint8_t edge_detector_is_edge_detected(void);

#endif /* EDGE_DETECTOR_H */
