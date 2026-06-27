#include "state_machine.h"

#include "edge_detector.h"
#include "distance_sensor.h"
#include "failsafe.h"
#include "motor_control.h"
#include "motion.h"
#include "opponent_tracker.h"
#include "robot_config.h"
#include "usart1_log.h"
#include "vesc/vescuart.h"

#define EDGE_TEST 1
#define OPPONENT_TEST 0

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define EDGE_ESCAPE_DURATION_MS 1000U
#define EDGE_ESCAPE_BACKUP_MS 600U
#define IR1_EDGE_TEST_ENABLE 0
#define IR2_EDGE_TEST_ENABLE 0
#define IR1_EDGE_DETECTED_STATE GPIO_PIN_RESET
#define IR2_EDGE_DETECTED_STATE GPIO_PIN_RESET
#define VESC_FAULT_POLL_PERIOD_MS 200U
#define VESC_FAULT_RECOVERY_WAIT_MS 500U

static uint32_t current_time = 0;
static uint32_t escape_start_time = 0;
static uint32_t last_vesc_fault_poll_time = 0;
static uint32_t vesc_fault_start_time = 0;
static volatile uint8_t is_escaping = 0;
static uint8_t edge_mode = 0;
static volatile uint8_t escape_timer_started = 0;
static uint8_t run_once = 0;
static uint8_t vesc_fault_latched = 0;
static volatile uint8_t ir1_interrupt_pending = 0;
static volatile uint8_t ir2_interrupt_pending = 0;
static volatile uint32_t ir1_interrupt_time_ms = 0;
static volatile uint32_t ir2_interrupt_time_ms = 0;

static robot_state_t current_state;
static volatile robot_edge_escape_mode_t current_escape_mode;

VescUart_t vesc1;
VescUart_t vesc2;

static void edge_debug_show_accepted(void)
{
	HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_SET);
}

static void edge_debug_clear_unaccepted(void)
{
	if (is_escaping == 0U)
	{
		HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_RESET);
	}
}

static void edge_escape_begin(robot_edge_escape_mode_t escape_mode)
{
	current_escape_mode = escape_mode;
	escape_timer_started = 0U;
	is_escaping = 1U;
	edge_debug_show_accepted();
}


static void edge_process_detection(void)
{
	edge_debug_clear_unaccepted();

	if (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) != GPIO_PIN_SET)
	{
		ir1_interrupt_pending = 0U;
		ir2_interrupt_pending = 0U;
		return;
	}

	if (is_escaping != 0U)
	{
		return;
	}

#if IR2_EDGE_TEST_ENABLE
	edge_process_interrupt_candidate(&ir2_interrupt_pending,
									 ir2_interrupt_time_ms,
									 IR2_DO_GPIO_Port,
									 IR2_DO_Pin,
									 IR2_EDGE_DETECTED_STATE,
									 ROBOT_ESCAPE_BACK_RIGHT);
#endif

	if (is_escaping != 0U)
	{
		return;
	}

#if IR1_EDGE_TEST_ENABLE
	edge_process_interrupt_candidate(&ir1_interrupt_pending,
									 ir1_interrupt_time_ms,
									 IR1_DO_GPIO_Port,
									 IR1_DO_Pin,
									 IR1_EDGE_DETECTED_STATE,
									 ROBOT_ESCAPE_BACK_RIGHT);
#endif

}

static void edge_process_analog_detection(const edge_status_t *edge)
{
	if (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) != GPIO_PIN_SET)
	{
		return;
	}

	if (is_escaping != 0U)
	{
		return;
	}

	if (edge->front_left != 0U)
	{
		edge_escape_begin(ROBOT_ESCAPE_BACK_RIGHT);
	}
	else if (edge->front_right != 0U)
	{
		edge_escape_begin(ROBOT_ESCAPE_BACK_LEFT);
	}
}

static void vesc_stop_all(void)
{
    VescUart_SetCurrent(&vesc1, 0.0f);
    VescUart_SetCurrent(&vesc2, 0.0f);
    VescUart_SetDuty(&vesc1, 0.0f);
    VescUart_SetDuty(&vesc2, 0.0f);
}

static uint8_t vesc_is_overcurrent_fault(mc_fault_code fault)
{
    return (uint8_t)(fault == FAULT_CODE_ABS_OVER_CURRENT);
}

static uint8_t vesc_check_overcurrent_fault(void)
{
    const uint32_t current_time = HAL_GetTick();

    if ((current_time - last_vesc_fault_poll_time) < VESC_FAULT_POLL_PERIOD_MS)
    {
        return 0U;
    }

    last_vesc_fault_poll_time = current_time;

    const uint8_t vesc1_values_ok = VescUart_GetVescValues(&vesc1) ? 1U : 0U;
    const uint8_t vesc2_values_ok = VescUart_GetVescValues(&vesc2) ? 1U : 0U;

    if (vesc1_values_ok && vesc_is_overcurrent_fault(vesc1.data.error))
    {
        LOG_PRINT("VESC1 overcurrent fault detected\r\n");
        return 1U;
    }

    if (vesc2_values_ok && vesc_is_overcurrent_fault(vesc2.data.error))
    {
        LOG_PRINT("VESC2 overcurrent fault detected\r\n");
        return 1U;
    }

    return 0U;
}

static uint8_t vesc_overcurrent_faults_clear(void)
{
    const uint8_t vesc1_values_ok = VescUart_GetVescValues(&vesc1) ? 1U : 0U;
    const uint8_t vesc2_values_ok = VescUart_GetVescValues(&vesc2) ? 1U : 0U;

    if (!vesc1_values_ok || !vesc2_values_ok)
    {
        return 0U;
    }

    return (uint8_t)(!vesc_is_overcurrent_fault(vesc1.data.error) &&
                     !vesc_is_overcurrent_fault(vesc2.data.error));
}

void state_machine_init(void)
{
//    VescUart_Init(&vesc1, &huart1, 100);
//    VescUart_Init(&vesc2, &huart2, 100);
//
//    vesc_stop_all();
    current_state = ROBOT_STATE_IDLE;
    is_escaping = 0;
    motor_control_stop();

    run_once = 0;
}

void state_machine_background(void)
{
}

void state_machine_update(void)
{
    const opponent_status_t opponent = opponent_tracker_get_status();
    const edge_status_t edge = edge_detector_get_status();

    //TOF_debug();

#if EDGE_TEST
    edge_process_analog_detection(&edge);
    edge_process_detection();
#endif

    if (failsafe_is_faulted())
    {
        current_state = ROBOT_STATE_FAULT;
        vesc_stop_all();
    }
    else if (run_once)
    {
        return;
    }
//    else if (vesc_fault_latched)
//    {
        //     current_state = ROBOT_STATE_FAULT;
        //     vesc_stop_all();

        //     if ((HAL_GetTick() - vesc_fault_start_time) >= VESC_FAULT_RECOVERY_WAIT_MS) {
        //         if (vesc_overcurrent_faults_clear()) {
        //             vesc_fault_latched = 0U;
        //             current_state = ROBOT_STATE_RECOVER;
        //             LOG_PRINT("VESC overcurrent recovered after 500 ms\r\n");
        //         } else {
        //             vesc_fault_start_time = HAL_GetTick();
        //             LOG_PRINT("VESC overcurrent still active. Recovery wait restarted\r\n");
        //         }
        //     }
//    }
#if EDGE_TEST
    else if (is_escaping)
    {
    	current_time = HAL_GetTick();

    	if (!escape_timer_started)
    	{
    		escape_start_time = current_time;
    		escape_timer_started = 1;
    	}

    	if (current_time - escape_start_time <= EDGE_ESCAPE_DURATION_MS)
    	{
    		current_state = ROBOT_STATE_EDGE_ESCAPE;
    	}
    	else
    	{
    		// VescUart_SetCurrent(&vesc1, -10.0f);
    		// VescUart_SetCurrent(&vesc2, -10.0f);
    		current_escape_mode = ROBOT_ESCAPE_NONE;
    		is_escaping = 0;
    		escape_timer_started = 0;
    		current_state = ROBOT_STATE_SEARCH;
    		//vesc_stop_all();
    		LOG_PRINT("Edge escape complete. VESC current stopped.\r\n");

    		// run_once = 1;
    		}
    	// } else if (vesc_check_overcurrent_fault()) {
    	// vesc_fault_latched = 1U;
    	// vesc_fault_start_time = HAL_GetTick();
    	// current_state = ROBOT_STATE_FAULT;
    	// vesc_stop_all();

    }
#endif
#if OPPONENT_TEST
    else if (opponent.front)
    {
        current_state = ROBOT_STATE_ATTACK;
    }
    else if (opponent.left || opponent.right || opponent.rear_left || opponent.rear_right)
    {
        current_state = ROBOT_STATE_SEARCH;
    }
#endif
    else
    {
        current_state = ROBOT_STATE_SEARCH;
    }


    switch (current_state)
    {

        #if EDGE_TEST
    case ROBOT_STATE_EDGE_ESCAPE:
        // motor_control_set_command(motion_reverse(ROBOT_EDGE_ESCAPE_PWM));
        // motor_control_set_pwm(2250, 2250);
        // VescUart_SetDuty(&vesc1, -0.94f);
        // VescUart_SetDuty(&vesc2, -0.94f);
        // VescUart_SetBrakeCurrent(&vesc1, 20.0f);
        // VescUart_SetBrakeCurrent(&vesc2, 20.0f);


        if (((current_escape_mode == ROBOT_ESCAPE_BACK_LEFT) ||
             (current_escape_mode == ROBOT_ESCAPE_BACK_RIGHT)) &&
            ((current_time - escape_start_time) <= EDGE_ESCAPE_BACKUP_MS))
        {
            motor_control_set_pwm(1000, 1000);
            break;
        }

        switch (current_escape_mode) {
            case ROBOT_ESCAPE_BACK:
                motor_control_set_pwm(900, 900);
                break;
            case ROBOT_ESCAPE_FRONT:
                motor_control_set_pwm(2250, 2250);
                break;
            case ROBOT_ESCAPE_BACK_LEFT:
                motor_control_set_pwm(1000, 1300);
                break;
            case ROBOT_ESCAPE_BACK_RIGHT:
                motor_control_set_pwm(1300, 1000);
                break;
            case ROBOT_ESCAPE_NONE:
            default:
            	motor_control_stop();
                break;
        }

        //LOG_PRINT("Edge detected! Escaping with VESC current\r\n");
        break;
        #endif

    #if OPPONENT_TEST
    case ROBOT_STATE_ATTACK:

        //motor_control_set_command(motion_forward(ROBOT_ATTACK_PWM));
    	int front_mm = front_mm_return();
    	if (front_mm <= 400){
    		motor_control_set_pwm(1950, 1950);
    	}
        //LOG_PRINT("Attacking\n");
        HAL_GPIO_WritePin(LED_D6_GPIO_Port,
        		LED_D6_Pin,
				GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_D7_GPIO_Port,
        		LED_D7_Pin,
				GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_D8_GPIO_Port,
        		LED_D8_Pin,
				GPIO_PIN_SET);
        break;

    case ROBOT_STATE_SEARCH:
        if (opponent.left)
        {
            // motor_control_set_command(motion_rotate_foward(ROBOT_TRACK_PWM));

        	motor_control_set_pwm(1600, 1850);
            //LOG_PRINT("Opponent on the left! Rotating left\n");

            HAL_GPIO_WritePin(LED_D6_GPIO_Port,
                      LED_D6_Pin,
                      GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_D7_GPIO_Port,
                      LED_D7_Pin,
					  GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_D8_GPIO_Port,
                      LED_D8_Pin,
					  GPIO_PIN_RESET);


        }
        else if (opponent.right)
        {
            //motor_control_set_command(motion_rotate_right(ROBOT_TRACK_PWM));

        	motor_control_set_pwm(1850, 1600);
            //LOG_PRINT("Opponent on the right! Rotating right\n");
            HAL_GPIO_WritePin(LED_D6_GPIO_Port,
            		LED_D6_Pin,
					GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_D7_GPIO_Port,
            		LED_D7_Pin,
					GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_D8_GPIO_Port,
            		LED_D8_Pin,
					GPIO_PIN_RESET);
        }
        else if (opponent.rear_left)
        {
        	motor_control_set_pwm(1850, 1150);
        }
        else if (opponent.rear_right)
        {
        	motor_control_set_pwm(1150, 1850);
        }
        else
        {
            // motor_control_set_command(motion_rotate_left(ROBOT_SEARCH_PWM));
            // VescUart_SetDuty(&vesc1, 0.94f);
            // VescUart_SetDuty(&vesc2, 0.94f);
            // VescUart_SetCurrent(&vesc1, 10.0f);
            // VescUart_SetCurrent(&vesc2, 10.0f);
            // motor_control_set_pwm(1700, 1700);
            // LOG_PRINT("No opponent detected! Searching\n");

        	motor_control_set_pwm(1500, 1500);

            HAL_GPIO_WritePin(LED_D6_GPIO_Port,
            		LED_D6_Pin,
					GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_D7_GPIO_Port,
            		LED_D7_Pin,
					GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_D8_GPIO_Port,
            		LED_D8_Pin,
					GPIO_PIN_SET);
        }
        break;
    #else
    case ROBOT_STATE_SEARCH:
        motor_control_set_pwm(2150, 2150);
        break;
    #endif

    case ROBOT_STATE_IDLE:
//    	motor_control_set_pwm(1500, 1500);
//    	break;
    case ROBOT_STATE_RECOVER:
    case ROBOT_STATE_FAULT:
    default:
        vesc_stop_all();
        motor_control_stop();
        break;
    }

    return;
}

robot_state_t state_machine_get_state(void)
{
    return current_state;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	const uint32_t now_ms = HAL_GetTick();

	if (HAL_GPIO_ReadPin(SM_Signal_GPIO_Port, SM_Signal_Pin) != GPIO_PIN_SET)
	{
		if (GPIO_Pin == IR2_DO_Pin)
		{
			ir2_interrupt_pending = 0U;
		}
		else if (GPIO_Pin == IR1_DO_Pin)
		{
			ir1_interrupt_pending = 0U;
		}
		return;
	}

	if (GPIO_Pin == IR2_DO_Pin)
	{
#if IR2_EDGE_TEST_ENABLE
		if (HAL_GPIO_ReadPin(IR2_DO_GPIO_Port, IR2_DO_Pin) == IR2_EDGE_DETECTED_STATE)
		{
			ir2_interrupt_time_ms = now_ms;
			ir2_interrupt_pending = 1U;
		}
		else
		{
			ir2_interrupt_pending = 0U;
		}
#endif
	}
	else if (GPIO_Pin == IR1_DO_Pin)
	{
#if IR1_EDGE_TEST_ENABLE
		if (HAL_GPIO_ReadPin(IR1_DO_GPIO_Port, IR1_DO_Pin) == IR1_EDGE_DETECTED_STATE)
		{
			ir1_interrupt_time_ms = now_ms;
			ir1_interrupt_pending = 1U;
		}
		else
		{
			ir1_interrupt_pending = 0U;
		}
#endif
	}
}
