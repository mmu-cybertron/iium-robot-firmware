#ifndef OPPONENT_TRACKER_H
#define OPPONENT_TRACKER_H

#include "robot_types.h"

void opponent_tracker_init(void);
void opponent_tracker_update(void);
opponent_status_t opponent_tracker_get_status(void);
uint8_t opponent_tracker_has_target(void);

#endif /* OPPONENT_TRACKER_H */
