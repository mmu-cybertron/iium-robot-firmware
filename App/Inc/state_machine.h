#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "robot_types.h"

void state_machine_init(void);
void state_machine_update(void);
robot_state_t state_machine_get_state(void);

#endif /* STATE_MACHINE_H */
