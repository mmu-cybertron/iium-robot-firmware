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

void robot_init(void)
{
    LOG_PRINT("\r\n=================================\r\n");
    LOG_PRINT("Sumo Robot FSM Booting...\r\n");
    LOG_PRINT("=================================\r\n");

    // Initialize all subsystems
    failsafe_init();
    edge_detector_init();
    opponent_tracker_init();
    motor_control_init();
    state_machine_init();

    LOG_PRINT("\r\n[SYSTEM] Auto FSM Ready and Running!\r\n");
}

void robot_update(void)
{
    // 1. Update all sensor readings
    failsafe_update();
    edge_detector_update();
    opponent_tracker_update();

    // 2. Run Autonomous State Machine continuously
    state_machine_update();

    // 3. Send the final chosen command to the physical motors
    motor_control_update();
}

void robot_background(void)
{
    // Background tasks can go here if needed later
}
