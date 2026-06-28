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

#define EDGE_TEST ROBOT_EDGE_SENSOR_ENABLE
#define OPPONENT_TEST 1

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
static uint32_t opponent_track_start_ms = 0;
static uint32_t opponent_front_last_seen_ms = 0;
static uint32_t opponent_left_cooldown_until_ms = 0;
static uint32_t opponent_right_cooldown_until_ms = 0;
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

static void opponent_debug_leds(const opponent_status_t *opponent)
{
#if ROBOT_TOF_FAILURE_LED_DIAG_ENABLE
	(void)opponent;
	return;
#else
	HAL_GPIO_WritePin(LED_D6_GPIO_Port,
					  LED_D6_Pin,
					  opponent->left ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_D7_GPIO_Port,
					  LED_D7_Pin,
					  opponent->right ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_D8_GPIO_Port,
					  LED_D8_Pin,
					  opponent->front ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static void edge_debug_show_accepted(void)
{
	HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_SET);
}

static void edge_debug_clear_unaccepted(void)
{
#if ROBOT_TOF_FAILURE_LED_DIAG_ENABLE
	return;
#else
	if (is_escaping == 0U)
	{
		HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_RESET);
	}
#endif
}

static void edge_escape_begin(robot_edge_escape_mode_t escape_mode)
{
	current_escape_mode = escape_mode;
	current_time = HAL_GetTick();
	escape_start_time = current_time;
	escape_timer_started = 1U;
	is_escaping = 1U;
	edge_debug_show_accepted();
}

static void edge_escape_drive_turn(robot_edge_escape_mode_t escape_mode)
{
	switch (escape_mode) {
		case ROBOT_ESCAPE_BACK_LEFT:
			motor_control_set_pwm(2150, 1000);
			break;
		case ROBOT_ESCAPE_BACK_RIGHT:
			motor_control_set_pwm(1000, 2150);
			break;
		case ROBOT_ESCAPE_BACK:
			motor_control_set_pwm(900, 900);
			break;
		case ROBOT_ESCAPE_FRONT:
			motor_control_set_pwm(2250, 2250);
			break;
		case ROBOT_ESCAPE_NONE:
		default:
			motor_control_stop();
			break;
	}
}

static void edge_escape_execute_blocking(void)
{
	const robot_edge_escape_mode_t escape_mode = current_escape_mode;
	const uint32_t turn_ms = EDGE_ESCAPE_DURATION_MS - EDGE_ESCAPE_BACKUP_MS;

	current_state = ROBOT_STATE_EDGE_ESCAPE;

    motor_control_set_pwm(1500, 1500);
    motor_control_update();
	motor_control_set_pwm(900, 900);
	motor_control_update();
	const uint32_t recovery_start_ms = HAL_GetTick();
	distance_sensor_recover_during_edge_escape();
	const uint32_t recovery_elapsed_ms = HAL_GetTick() - recovery_start_ms;
	if (recovery_elapsed_ms < EDGE_ESCAPE_BACKUP_MS) {
		HAL_Delay(EDGE_ESCAPE_BACKUP_MS - recovery_elapsed_ms);
	}

	edge_escape_drive_turn(escape_mode);
	motor_control_update();
	HAL_Delay(turn_ms);

	current_escape_mode = ROBOT_ESCAPE_NONE;
	is_escaping = 0U;
	escape_timer_started = 0U;
	current_state = ROBOT_STATE_SEARCH;
	motor_control_set_pwm(2150, 2150);
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
    opponent_track_start_ms = 0U;
    opponent_front_last_seen_ms = 0U;
    opponent_left_cooldown_until_ms = 0U;
    opponent_right_cooldown_until_ms = 0U;
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
    const uint32_t now_ms = HAL_GetTick();
    uint8_t front_seen_or_latched = opponent.front;

    if (opponent.front != 0U) {
        opponent_front_last_seen_ms = now_ms;
    } else if ((opponent_front_last_seen_ms != 0U) &&
               ((now_ms - opponent_front_last_seen_ms) <= OPPONENT_FRONT_LATCH_MS)) {
        front_seen_or_latched = 1U;
    }

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
    	edge_escape_execute_blocking();
    	return;
    }
#endif
#if OPPONENT_TEST
    else if (front_seen_or_latched)
    {
        current_state = ROBOT_STATE_ATTACK;
    }
    else if ((current_state == ROBOT_STATE_TRACK_LEFT) ||
             (current_state == ROBOT_STATE_TRACK_RIGHT))
    {
    	if ((now_ms - opponent_track_start_ms) >= OPPONENT_TRACK_TIMEOUT_MS)
    	{
            if (current_state == ROBOT_STATE_TRACK_LEFT)
            {
                opponent_left_cooldown_until_ms = now_ms + OPPONENT_TRACK_COOLDOWN_MS;
            }
            else
            {
                opponent_right_cooldown_until_ms = now_ms + OPPONENT_TRACK_COOLDOWN_MS;
            }
    		current_state = ROBOT_STATE_SEARCH;
    	}
    }
    else if ((opponent.left != 0U) &&
             ((int32_t)(now_ms - opponent_left_cooldown_until_ms) >= 0))
    {
        current_state = ROBOT_STATE_TRACK_LEFT;
        opponent_track_start_ms = now_ms;
    }
    else if ((opponent.right != 0U) &&
             ((int32_t)(now_ms - opponent_right_cooldown_until_ms) >= 0))
    {
        current_state = ROBOT_STATE_TRACK_RIGHT;
        opponent_track_start_ms = now_ms;
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
    	// motor_control_stop();
        break;
        #endif

    #if OPPONENT_TEST
    case ROBOT_STATE_ATTACK:

        //motor_control_set_command(motion_forward(ROBOT_ATTACK_PWM));
    	int front_mm = front_mm_return();
    	if (front_mm <= 700){
    		motor_control_set_pwm(2250, 2250);
    	}
        //LOG_PRINT("Attacking\n");
        opponent_debug_leds(&opponent);
        break;

    case ROBOT_STATE_TRACK_LEFT:
    	opponent_debug_leds(&opponent);
        motor_control_set_pwm(2150, 1000);
        break;

    case ROBOT_STATE_TRACK_RIGHT:
    	opponent_debug_leds(&opponent);
        motor_control_set_pwm(1000, 2150);
        break;

    case ROBOT_STATE_SEARCH:
    	opponent_debug_leds(&opponent);

        motor_control_set_pwm(2150, 2150);
        break;
    #else
    case ROBOT_STATE_SEARCH:
        motor_control_set_pwm(2250, 2250);
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
#if IR1_EDGE_TEST_ENABLE || IR2_EDGE_TEST_ENABLE
	const uint32_t now_ms = HAL_GetTick();
#endif

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
