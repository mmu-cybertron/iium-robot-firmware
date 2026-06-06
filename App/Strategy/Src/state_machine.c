#include "state_machine.h"

#include "edge_detector.h"
#include "failsafe.h"
#include "motor_control.h"
#include "motion.h"
#include "opponent_tracker.h"

static robot_state_t current_state;

void state_machine_init(void)
{
    current_state = ROBOT_STATE_IDLE;
    motor_control_stop();
}

void state_machine_update(void)
{
    if (failsafe_is_faulted()) {
        current_state = ROBOT_STATE_FAULT;
    } else if (edge_detector_is_edge_detected()) {
        current_state = ROBOT_STATE_EDGE_ESCAPE;
    } else if (opponent_tracker_has_target()) {
        current_state = ROBOT_STATE_ATTACK;
    } else {
        current_state = ROBOT_STATE_SEARCH;
    }

    switch (current_state) {
    case ROBOT_STATE_ATTACK:
        motor_control_set_command(motion_forward(700));
        break;

    case ROBOT_STATE_EDGE_ESCAPE:
        motor_control_set_command(motion_reverse(600));
        break;

    case ROBOT_STATE_SEARCH:
        motor_control_set_command(motion_rotate_left(350));
        break;

    case ROBOT_STATE_IDLE:
    case ROBOT_STATE_START_DELAY:
    case ROBOT_STATE_RECOVER:
    case ROBOT_STATE_FAULT:
    default:
        motor_control_stop();
        break;
    }
}

robot_state_t state_machine_get_state(void)
{
    return current_state;
}
