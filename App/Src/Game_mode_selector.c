#include "game_mode_selector.h"

#include "main.h"
#include "motor_control.h"
#include "robot_config.h"
#include "usart1_log.h"

extern uint32_t HAL_GetTick(void);

/* Button pin definitions from main.h */
/* Mode_Button_Pin = GPIO_PIN_12 (GPIOB)
 * Confirm_Button_Pin = GPIO_PIN_13 (GPIOB)
 */

/* LED pin mapping (defined in main.h):
 * GAME_MODE_1 -> LED_D7 (GPIOB, PIN_15)
 * GAME_MODE_2 -> LED_D6 (GPIOB, PIN_14)
 * GAME_MODE_3 -> LED_D8 (GPIOA, PIN_8)
 */

/* Timing constants (in milliseconds) */
#define LONG_PRESS_THRESHOLD_MS  2000
#define DEBOUNCE_MS              20
#define INITIAL_MOVE_MODE1_DURATION_MS   2000  // Turn left 1s + forward 1s
#define INITIAL_MOVE_MODE2_DURATION_MS   2000  // Turn right 1s + forward 1s
#define INITIAL_MOVE_MODE3_DURATION_MS   1000  // Forward 1s
#define TURN_DURATION_MS         1000
#define FORWARD_DURATION_MS      1000

/* Motor command definitions
 *
 * IMPORTANT: motor_driver_set_pwm() writes these values DIRECTLY into the
 * timer compare register (see motor_driver.c) - there is no sign handling,
 * no scaling, and the DIR GPIO pins are never touched by the driver.
 * Direction comes purely from where the value sits relative to
 * MOTOR_PWM_NEUTRAL, same convention used in state_machine.c
 * (e.g. ROBOT_ESCAPE_LEFT -> motor_control_set_pwm(1600, 2200)).
 *
 * PWM_TURN_OFFSET / PWM_FORWARD_OFFSET below are PLACEHOLDERS based on the
 * literal values already used in state_machine.c (900-2250 range, neutral
 * ~1500). Replace with your actual tuned offsets if different - what
 * matters is that all values are expressed relative to MOTOR_PWM_NEUTRAL,
 * not as small signed numbers around 0.
 */
#define PWM_FORWARD_OFFSET     250
#define PWM_TURN_OFFSET        200

#define MOTOR_FORWARD_PWM       (MOTOR_PWM_NEUTRAL + PWM_FORWARD_OFFSET)
#define MOTOR_TURN_LEFT_PWM_L   (MOTOR_PWM_NEUTRAL - PWM_TURN_OFFSET)
#define MOTOR_TURN_LEFT_PWM_R   (MOTOR_PWM_NEUTRAL + PWM_TURN_OFFSET)
#define MOTOR_TURN_RIGHT_PWM_L  (MOTOR_PWM_NEUTRAL + PWM_TURN_OFFSET)
#define MOTOR_TURN_RIGHT_PWM_R  (MOTOR_PWM_NEUTRAL - PWM_TURN_OFFSET)

/* State variables */
static mode_selector_state_t current_state = MODE_SEL_IDLE;
static game_mode_t selected_mode = GAME_MODE_1;
static uint32_t button_press_start_time = 0;
static uint8_t pb12_was_pressed = 0;
static uint8_t pb13_was_pressed = 0;
static uint32_t initial_move_start_time = 0;
static uint8_t initial_move_in_progress = 0;
static uint8_t initial_move_phase = 0;  // 0=idle, 1=turn phase, 2=forward phase

/* Helper function: Update LED indicators based on selected mode */
static void update_mode_leds(void)
{
    /* Turn off all LEDs first */
    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_RESET);

    /* Turn on LED for selected mode */
    switch (selected_mode) {
        case GAME_MODE_1:
            HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_SET);
            break;
        case GAME_MODE_2:
            HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_SET);

            break;
        case GAME_MODE_3:
            HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_SET);

            break;
        default:
            break;
    }
}

/* Helper function: Turn on all LEDs (confirmation state) */
static void turn_on_all_leds(void)
{
    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_SET);
}

/* Helper function: Turn off all LEDs */
static void turn_off_all_leds(void)
{
    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_RESET);
}

/* Helper function: Read button states (GPIO_PIN_RESET = pressed, button pulls pin low) */
static uint8_t is_pb12_pressed(void)
{
    return HAL_GPIO_ReadPin(Mode_Button_GPIO_Port, Mode_Button_Pin) == GPIO_PIN_RESET;
}

static uint8_t is_pb13_pressed(void)
{
    return HAL_GPIO_ReadPin(Confirm_Button_GPIO_Port, Confirm_Button_Pin) == GPIO_PIN_RESET;
}

/* Helper function: Cycle to next mode */
static void cycle_to_next_mode(void)
{
    if (selected_mode == GAME_MODE_1) {

        selected_mode = GAME_MODE_2;
    } else if (selected_mode == GAME_MODE_2) {


        selected_mode = GAME_MODE_3;
    } else {


        selected_mode = GAME_MODE_1;
    }
    update_mode_leds();
    LOG_PRINT("Mode changed to: %d\r\n", (int)selected_mode);
}

/* Initialize mode selector */
void game_mode_selector_init(void)
{
    current_state = MODE_SEL_IDLE;
    selected_mode = GAME_MODE_1;
    button_press_start_time = 0;
    pb12_was_pressed = 0;
    pb13_was_pressed = 0;
    initial_move_in_progress = 0;
    initial_move_phase = 0;

    update_mode_leds();
    LOG_PRINT("Mode selector initialized. Select mode with PB12, confirm with PB13 (long-press 2s)\r\n");
}

/* Main update function - call this regularly (every ~10-20ms) during mode selection */
void game_mode_selector_update(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t pb12_pressed = is_pb12_pressed();
    uint8_t pb13_pressed = is_pb13_pressed();

    switch (current_state) {
        case MODE_SEL_IDLE:
        case MODE_SEL_SELECTING:
            /* Handle PB12 short click (mode cycling) */
            if (pb12_pressed && !pb12_was_pressed) {
                pb12_was_pressed = 1;
            	LOG_PRINT(" 2 \r\n");

                current_state = MODE_SEL_SELECTING;
                cycle_to_next_mode();
            }
            if (!pb12_pressed) {
                pb12_was_pressed = 0;

            }

            /* Handle PB13 long-press (confirmation) */
            if (pb13_pressed && !pb13_was_pressed) {
                pb13_was_pressed = 1;
                button_press_start_time = now_ms;
            	LOG_PRINT(" 1: \r\n");

            }
            if (pb13_pressed && pb13_was_pressed) {
                if ((now_ms - button_press_start_time) >= LONG_PRESS_THRESHOLD_MS) {
                    current_state = MODE_SEL_LOCKED;
                    turn_on_all_leds();
                    LOG_PRINT("Mode %d LOCKED and ready!\r\n", (int)selected_mode);
                    pb13_was_pressed = 0;
                }
            }
            if (!pb13_pressed) {
                pb13_was_pressed = 0;

            }
            break;

        case MODE_SEL_LOCKED:
            /* In locked state, only PB13 long-press can unlock */
            if (pb13_pressed && !pb13_was_pressed) {
                pb13_was_pressed = 1;
                button_press_start_time = now_ms;
            }
            if (pb13_pressed && pb13_was_pressed) {
                if ((now_ms - button_press_start_time) >= LONG_PRESS_THRESHOLD_MS) {
                    current_state = MODE_SEL_IDLE;
                    selected_mode = GAME_MODE_1;
                    turn_off_all_leds();
                    update_mode_leds();
                    LOG_PRINT("Mode unlocked. Re-selecting...\r\n");
                    pb13_was_pressed = 0;
                }
            }
            if (!pb13_pressed) {
                pb13_was_pressed = 0;
            }
            break;

        default:
            break;
    }
}

/* Check if mode is locked and ready */
uint8_t game_mode_selector_is_locked(void)
{
    return (current_state == MODE_SEL_LOCKED);
}

/* Get the selected mode */
game_mode_t game_mode_selector_get_mode(void)
{
    return selected_mode;
}

/* Execute initial move based on selected mode */
void game_mode_selector_execute_initial_move(void)
{
    uint32_t now_ms = HAL_GetTick();
    motor_command_t cmd;

    if (!initial_move_in_progress) {
        initial_move_start_time = now_ms;
        initial_move_in_progress = 1;
        initial_move_phase = 1;
        LOG_PRINT("Starting initial move - Mode %d\r\n", (int)selected_mode);
        return;
    }

    uint32_t elapsed_ms = now_ms - initial_move_start_time;

    switch (selected_mode) {
        case GAME_MODE_1:
            /* Turn left 1s, then move forward 1s */
            if (initial_move_phase == 1) {
                if (elapsed_ms < TURN_DURATION_MS) {
                    /* Turn left */
                    cmd.left_pwm = MOTOR_TURN_LEFT_PWM_L;
                    cmd.right_pwm = MOTOR_TURN_LEFT_PWM_R;
                    motor_control_set_command(cmd);
                } else {
                    initial_move_phase = 2;
                    initial_move_start_time = now_ms;
                }
            } else if (initial_move_phase == 2) {
                elapsed_ms = now_ms - initial_move_start_time;
                if (elapsed_ms < FORWARD_DURATION_MS) {
                    /* Move forward */
                    cmd.left_pwm = MOTOR_FORWARD_PWM;
                    cmd.right_pwm = MOTOR_FORWARD_PWM;
                    motor_control_set_command(cmd);
                } else {
                    /* Done */
                    motor_control_stop();
                    initial_move_in_progress = 0;
                    initial_move_phase = 0;
                    LOG_PRINT("Initial move complete - Mode 1\r\n");
                }
            }
            break;

        case GAME_MODE_2:
            /* Turn right 1s, then move forward 1s */
            if (initial_move_phase == 1) {
                if (elapsed_ms < TURN_DURATION_MS) {
                    /* Turn right */
                    cmd.left_pwm = MOTOR_TURN_RIGHT_PWM_L;
                    cmd.right_pwm = MOTOR_TURN_RIGHT_PWM_R;
                    motor_control_set_command(cmd);
                } else {
                    initial_move_phase = 2;
                    initial_move_start_time = now_ms;
                }
            } else if (initial_move_phase == 2) {
                elapsed_ms = now_ms - initial_move_start_time;
                if (elapsed_ms < FORWARD_DURATION_MS) {
                    /* Move forward */
                    cmd.left_pwm = MOTOR_FORWARD_PWM;
                    cmd.right_pwm = MOTOR_FORWARD_PWM;
                    motor_control_set_command(cmd);
                } else {
                    /* Done */
                    motor_control_stop();
                    initial_move_in_progress = 0;
                    initial_move_phase = 0;
                    LOG_PRINT("Initial move complete - Mode 2\r\n");
                }
            }
            break;

        case GAME_MODE_3:
            /* Move forward 1s only */
            if (elapsed_ms < INITIAL_MOVE_MODE3_DURATION_MS) {
                cmd.left_pwm = MOTOR_FORWARD_PWM;
                cmd.right_pwm = MOTOR_FORWARD_PWM;
                motor_control_set_command(cmd);
            } else {
                /* Done */
                motor_control_stop();
                initial_move_in_progress = 0;
                initial_move_phase = 0;
                LOG_PRINT("Initial move complete - Mode 3\r\n");
            }
            break;

        default:
            break;
    }
}

/* Check if initial move is still in progress */
uint8_t game_mode_selector_is_initial_move_done(void)
{
    return (initial_move_in_progress == 0);
}

/* Reset for new round */
void game_mode_selector_reset_for_new_round(void)
{
    current_state = MODE_SEL_IDLE;
    selected_mode = GAME_MODE_1;
    button_press_start_time = 0;
    pb12_was_pressed = 0;
    pb13_was_pressed = 0;
    initial_move_in_progress = 0;
    initial_move_phase = 0;

    turn_off_all_leds();
    update_mode_leds();
    LOG_PRINT("Mode selector reset for new round\r\n");
}
