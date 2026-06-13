#include "robot.h"
#include "robot_config.h"
#include "main.h"

#include "motion.h"
#include "edge_detector.h"
#include "failsafe.h"
#include "motor_control.h"
#include "opponent_tracker.h"
#include "state_machine.h"
#include "usart1_log.h"

void robot_init(void)
{
	LOG_PRINT("Hello from init\n");
    // motor_control_init();
    edge_detector_init();
    // opponent_tracker_init();
    // failsafe_init();
    state_machine_init();

    LOG_PRINT("Robot initialized\r\n");
}

// Where the robot reads sensors, decides behavior, and updates motor PWM.
// Examples: read sensors, update edge detection, update opponent detection, choose movement, update motors

void robot_update(void)
{
    // failsafe_update();
    
    // if (failsafe_is_faulted()) {
    //     motor_control_stop();
    //     motor_control_update();
    //     return;
    // }

    edge_detector_update();
    // opponent_tracker_update();
    // // LOG_PRINT("In LOOP\r\n");

    

    
    state_machine_update();
    
    // motor_control_update();
}

// Call as often as possible inside the infinite loop. It is for non-timing-critical background tasks.
// Examples: checking communication, debug LED blinking, low-priority monitoring, background failsafe work, telemetry later

void robot_background(void)
{
    failsafe_background();
}


