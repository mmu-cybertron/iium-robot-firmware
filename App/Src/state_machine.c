#include "state_machine.h"

#include "edge_detector.h"
#include "failsafe.h"
#include "motor_control.h"
#include "motion.h"
#include "opponent_tracker.h"
#include "robot_config.h"
#include "usart1_log.h"

static robot_state_t current_state;

void state_machine_init(void)
{
    current_state = ROBOT_STATE_IDLE;
    motor_control_stop();
}

void state_machine_update(void)
{
    const opponent_status_t opponent = opponent_tracker_get_status();

    if (failsafe_is_faulted()) {
        current_state = ROBOT_STATE_FAULT;
    } else if (edge_detector_is_edge_detected()) {
        current_state = ROBOT_STATE_EDGE_ESCAPE;
    } else if (opponent.front) {
        current_state = ROBOT_STATE_ATTACK;
    } else if (opponent.left || opponent.right) {
        current_state = ROBOT_STATE_SEARCH;
    } else {
        current_state = ROBOT_STATE_SEARCH;
    }

    switch (current_state) {
    case ROBOT_STATE_ATTACK:
        
        motor_control_set_command(motion_forward(ROBOT_ATTACK_PWM));
        LOG_PRINT("Attacking\n");
        break;

    case ROBOT_STATE_EDGE_ESCAPE:
        motor_control_set_command(motion_reverse(ROBOT_EDGE_ESCAPE_PWM));
        LOG_PRINT("Edge detected! Escaping with PWM: %d\r\n", ROBOT_EDGE_ESCAPE_PWM);
        break;

    case ROBOT_STATE_SEARCH:
        if (opponent.left) {
            motor_control_set_command(motion_rotate_left(ROBOT_TRACK_PWM));
            LOG_PRINT("Opponent on the left! Rotating left\n");
        } else if (opponent.right) {
            motor_control_set_command(motion_rotate_right(ROBOT_TRACK_PWM));
            LOG_PRINT("Opponent on the right! Rotating right\n");
        } else {
            motor_control_set_command(motion_rotate_left(ROBOT_SEARCH_PWM));
            LOG_PRINT("No opponent detected! Searching\n"); 
        }
        break;

    case ROBOT_STATE_IDLE:
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
