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
}

void robot_update(void)
{
    static uint8_t motor_test_done;

    if (motor_test_done) {
        return;
    }

    failsafe_update();
    edge_detector_update();
    opponent_tracker_update();

    //edge_detector_update();
    //  opponent_tracker_update();
     LOG_PRINT("HI\n");
     


    motor_control_set_pwm(900, 900);
    motor_control_update();
    
    state_machine_update();
    
   
     HAL_Delay(1000);

     motor_control_set_pwm(2250, 2250);
     motor_control_update();

     HAL_Delay(1000);

     motor_control_set_pwm(900, 900);
    motor_control_update();
    
    // LOG_PRINT("HI2\n");
    // // state_machine_update();
    
   
     HAL_Delay(1000);

     motor_control_set_pwm(2250, 2250);
     motor_control_update();

    // HAL_Delay(1000);

    motor_control_stop();
    motor_test_done = 1U;
    LOG_PRINT("Motor test complete\r\n");
}

void robot_background(void)
{
    // Background tasks can go here if needed later
}
