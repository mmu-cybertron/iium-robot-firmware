#include "robot.h"
#include "robot_config.h"
#include "main.h"
#include "usart1_log.h"
#include "failsafe.h"
#include "edge_detector.h"
#include "opponent_tracker.h"
#include "motor_control.h"
#include "state_machine.h"
#include "motion.h"

extern UART_HandleTypeDef huart1;

// Start in manual mode so the robot doesn't instantly run off the desk!
static uint8_t is_manual_mode = 1U;

void robot_init(void)
{
    LOG_PRINT("\r\n=================================\r\n");
    LOG_PRINT("Sumo Robot FSM Booting...\r\n");
    LOG_PRINT("=================================\r\n");

    // Initialize all subsystems
    // (opponent_tracker_init automatically initializes all 5 distance sensors!)
    failsafe_init();
    edge_detector_init();
    opponent_tracker_init();
    motor_control_init();
    state_machine_init();

    LOG_PRINT("\r\n[SYSTEM] Ready!\r\n");
    LOG_PRINT(">>> STARTING IN MANUAL MODE <<<\r\n");
    LOG_PRINT("Press 'm' to toggle AUTO FSM / MANUAL\r\n");
    LOG_PRINT("Keys: 'w'=Fwd, 's'=Rev, 'a'=Left, 'd'=Right, ' '=Brake\r\n");
}

void robot_update(void)
{
    // 1. Update all sensor readings
    failsafe_update();
    edge_detector_update();
    opponent_tracker_update();

    // 2. Process UART for Mode Switch & Manual Override
    uint8_t rx_char = 0;

    // Timeout of 0 means this does NOT freeze the robot waiting for a key press
    if (HAL_UART_Receive(&huart1, &rx_char, 1, 0) == HAL_OK) {

        // Toggle between Manual and Autonomous
        if (rx_char == 'm' || rx_char == 'M') {
            is_manual_mode = !is_manual_mode;
            LOG_PRINT("\r\n>>> MODE: %s <<<\r\n", is_manual_mode ? "MANUAL" : "AUTO FSM");
            motor_control_stop();
        }

        // Manual Driving Controls
        if (is_manual_mode) {
            switch(rx_char) {
                case 'w':
                    motor_control_set_command(motion_forward(500));
                    LOG_PRINT("MANUAL: Forward\r\n");
                    break;
                case 's':
                    motor_control_set_command(motion_reverse(500));
                    LOG_PRINT("MANUAL: Reverse\r\n");
                    break;
                case 'a':
                    motor_control_set_command(motion_rotate_left(400));
                    LOG_PRINT("MANUAL: Left\r\n");
                    break;
                case 'd':
                    motor_control_set_command(motion_rotate_right(400));
                    LOG_PRINT("MANUAL: Right\r\n");
                    break;
                case ' ':
                    motor_control_stop();
                    LOG_PRINT("MANUAL: Brake\r\n");
                    break;
            }
        }
    }

    // 3. Run Autonomous State Machine ONLY if NOT in manual mode
    if (!is_manual_mode) {
        state_machine_update();
    }

    // 4. Send the final chosen command to the physical motors
    motor_control_update();
}

void robot_background(void)
{
    // Background tasks can go here if needed later
}
