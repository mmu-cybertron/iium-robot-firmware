#include "state_machine.h"

#include "edge_detector.h"
#include "failsafe.h"
#include "motor_control.h"
#include "motion.h"
#include "opponent_tracker.h"
#include "robot_config.h"
#include "usart1_log.h"
#include "vesc/vescuart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define EDGE_ESCAPE_DURATION_MS 1000U

static uint32_t escape_start_time = 0;
static uint8_t is_escaping = 0;
static uint8_t run_once = 0;

static robot_state_t current_state;

VescUart_t vesc1;
VescUart_t vesc2;

void state_machine_init(void)
{
    VescUart_Init(&vesc1, &huart1, 100);
    VescUart_Init(&vesc2, &huart2, 100);

    current_state = ROBOT_STATE_IDLE;
    motor_control_stop();
}

void state_machine_update(void)
{
    if (!run_once) {
        const opponent_status_t opponent = opponent_tracker_get_status();

    if (failsafe_is_faulted()) {
        current_state = ROBOT_STATE_FAULT;
    } else if (edge_detector_is_edge_detected()) {
    	uint32_t current_time = HAL_GetTick();

    	if (!is_escaping)
    	{
    		escape_start_time = current_time;
    		is_escaping = 1;
    	}

    	if (current_time - escape_start_time <= EDGE_ESCAPE_DURATION_MS)
    	{
    		current_state = ROBOT_STATE_EDGE_ESCAPE;
    	} else {
    		is_escaping = 0;
    		current_state = ROBOT_STATE_SEARCH;

    		run_once = 1;
    	}
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
        //motor_control_set_command(motion_reverse(ROBOT_EDGE_ESCAPE_PWM));
    	VescUart_SetDuty(&vesc1, -0.9f);
    	VescUart_SetDuty(&vesc2, 0.9f);
        LOG_PRINT("Edge detected! Escaping with PWM: %d\r\n", ROBOT_EDGE_ESCAPE_PWM);
        break;

    case ROBOT_STATE_SEARCH:
        if (opponent.left) {
            //motor_control_set_command(motion_rotate_left(ROBOT_TRACK_PWM));
            LOG_PRINT("Opponent on the left! Rotating left\n");
        } else if (opponent.right) {
            motor_control_set_command(motion_rotate_right(ROBOT_TRACK_PWM));
            LOG_PRINT("Opponent on the right! Rotating right\n");
        } else {
            motor_control_set_command(motion_rotate_left(ROBOT_SEARCH_PWM));
            VescUart_SetDuty(&vesc1, 0.9f);
            VescUart_SetDuty(&vesc2, 0.9f);
            // LOG_PRINT("No opponent detected! Searching\n"); 
        }
        break;

        case ROBOT_STATE_IDLE:
        case ROBOT_STATE_RECOVER:
        case ROBOT_STATE_FAULT:
        default:
            motor_control_stop();
            break;
        }

        return;
    }
}

robot_state_t state_machine_get_state(void)
{
    return current_state;
}
