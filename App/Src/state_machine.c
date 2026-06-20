#include "state_machine.h"

#include "edge_detector.h"
#include "failsafe.h"
#include "motor_control.h"
#include "motion.h"
#include "opponent_tracker.h"
#include "robot_config.h"
#include "usart1_log.h"

static robot_state_t current_state;
static robot_state_t previous_state = ROBOT_STATE_FAULT; // Used to track state changes

static const char* state_names[] = {
		    [ROBOT_STATE_IDLE]        = "IDLE",
		    [ROBOT_STATE_SEARCH]      = "SEARCH",
			[ROBOT_STATE_TRACK_LEFT]  = "TRACK_L",
			[ROBOT_STATE_TRACK_RIGHT] = "TRACK_R",
		    [ROBOT_STATE_ATTACK]      = "ATTACK",
		    [ROBOT_STATE_EDGE_ESCAPE] = "EDGE_ESCAPE",
		    [ROBOT_STATE_RECOVER]     = "RECOVER",
		    [ROBOT_STATE_FAULT]       = "FAULT"};


void state_machine_init(void)
{
    current_state = ROBOT_STATE_IDLE;
    motor_control_stop();
}

void state_machine_update(void)
{
    const opponent_status_t opponent = opponent_tracker_get_status();

    // 1. Determine the Next State based on Priorities
//    if (failsafe_is_faulted()) {
//        current_state = ROBOT_STATE_FAULT;
//    } else if (edge_detector_is_edge_detected()) {
//        current_state = ROBOT_STATE_EDGE_ESCAPE;
    if (opponent.front) {
            current_state = ROBOT_STATE_ATTACK;
        } else if (opponent.left || opponent.rear_left) {
            current_state = ROBOT_STATE_TRACK_LEFT;
        } else if (opponent.right || opponent.rear_right) {
            current_state = ROBOT_STATE_TRACK_RIGHT;
        } else {
            current_state = ROBOT_STATE_SEARCH;
        }

    // 2. Log State Changes to Tera Term (Only prints when the state actually changes!)
    if (current_state != previous_state) {
        LOG_PRINT("[FSM] State Changed: %s\r\n", state_names[current_state]);
        previous_state = current_state;
    }


    // --- SENSOR DASHBOARD DEBUG BLOCK ---
        // Prints twice a second so you can see EXACTLY what triggers!
        static uint32_t last_debug_ms = 0;
        if (HAL_GetTick() - last_debug_ms > 100) {
            LOG_PRINT("[DEBUG] State: %-7s | F:%d | L:%d R:%d | RL:%d RR:%d\r\n",
                      state_names[current_state],
                      opponent.front,
                      opponent.left, opponent.right,
                      opponent.rear_left, opponent.rear_right);
            last_debug_ms = HAL_GetTick();
        }
        // ------------------------------------

        // 3. Execute Motor Commands
        switch (current_state) {
            case ROBOT_STATE_ATTACK:
                motor_control_set_command(motion_forward(ROBOT_ATTACK_PWM));
                break;

            case ROBOT_STATE_EDGE_ESCAPE:
                motor_control_set_command(motion_reverse(ROBOT_EDGE_ESCAPE_PWM));
                break;

            case ROBOT_STATE_TRACK_LEFT:
                // Turn left to face the target on the left/rear-left
                motor_control_set_command(motion_rotate_left(ROBOT_TRACK_PWM));
                break;

            case ROBOT_STATE_TRACK_RIGHT:
                // Turn right to face the target on the right/rear-right
                motor_control_set_command(motion_rotate_right(ROBOT_TRACK_PWM));
                break;

            case ROBOT_STATE_SEARCH:
                // Default behavior if NO targets are seen: spin left continuously
                motor_control_set_command(motion_rotate_left(ROBOT_SEARCH_PWM));
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
