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
#include "vesc/vescuart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void robot_init(void)
{
    LOG_PRINT("Hello from init\n");

    motor_control_init();
    // HAL_Delay(1000);
    edge_detector_init();
    // opponent_tracker_init();
    failsafe_init();
    state_machine_init();
    HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_SET);
    LOG_PRINT("Robot initialized\r\n");
}

// Where the robot reads sensors, decides behavior, and updates motor PWM.
// Examples: read sensors, update edge detection, update opponent detection, choose movement, update motors

void robot_update(void)
{
    static uint8_t motor_test_done;

    if (!motor_test_done)
    {
        // return;
        failsafe_update();

        if (failsafe_is_faulted())
        {
            motor_control_stop();
            motor_control_update();
            return;
        }

        edge_detector_update();

        state_machine_update();

        // motor_control_set_pwm(900, 900);
        // HAL_Delay(1000);

        motor_control_update();

        // motor_control_set_pwm(2250, 2250);
        // HAL_Delay(1000);

        // motor_control_update();
    }
}

// Call as often as possible inside the infinite loop. It is for non-timing-critical background tasks.
// Examples: checking communication, debug LED blinking, low-priority monitoring, background failsafe work, telemetry later

void robot_background(void)
{
    failsafe_background();
}
