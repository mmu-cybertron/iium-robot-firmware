#include "robot.h"
#include "robot_config.h"
#include "main.h"
#include "usart1_log.h"
#include "failsafe.h"
#include "edge_detector.h"
#include "opponent_tracker.h"
#include "motor_control.h"
#include "state_machine.h"
#include "usart1_log.h"
#include "vesc/vescuart.h"

#define VESC_SETUP 0

void robot_init(void)
{
	LOG_PRINT("Hello from init\n");
    
    motor_control_init();
    HAL_Delay(1000);
    edge_detector_init();
    //opponent_tracker_init();
    failsafe_init();
    state_machine_init();
    //vl53l0x_init_rear_sensors();

    LOG_PRINT("Robot initialized\r\n");
}

void robot_update(void)
{
//    static uint8_t motor_test_done;
//
//    if (motor_test_done) {
//        return;
//    }
//
//    failsafe_update();
//
//    if (failsafe_is_faulted()) {
//        motor_control_stop();
//        motor_control_update();
//        return;
//    }
//
//    //edge_detector_update();
//    //  opponent_tracker_update();
//     LOG_PRINT("HI\n");
//
//    motor_control_set_pwm(900, 900);
//    motor_control_update();
//
//    // LOG_PRINT("HI2\n");
//    // // state_machine_update();
//
//     HAL_Delay(1000);
//
//     motor_control_set_pwm(2250, 2250);
//     motor_control_update();
//
//     HAL_Delay(1000);
//
//     motor_control_set_pwm(900, 900);
//    motor_control_update();
//
//    // LOG_PRINT("HI2\n");
//    // // state_machine_update();
//
//     HAL_Delay(1000);
//
//     motor_control_set_pwm(2250, 2250);
//     motor_control_update();
//
//    // HAL_Delay(1000);
//
//    motor_control_stop();
//    motor_test_done = 1U;
//    LOG_PRINT("Motor test complete\r\n");

    #if VESC_SETUP
        motor_control_set_pwm(2250, 2250);
        motor_control_update();
        HAL_Delay(1000);

        motor_control_set_pwm(900, 900);
        motor_control_update();
        HAL_Delay(1000);
    #else

    failsafe_update();

    if (failsafe_is_faulted())
    {
    	motor_control_stop();
    	motor_control_update();
    	return;
    }

    edge_detector_update();

    //opponent_tracker_update();
    
    state_machine_update();

    motor_control_update();
    #endif
}

void robot_background(void)
{
    // Background tasks can go here if needed later
}

