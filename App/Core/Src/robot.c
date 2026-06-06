#include "robot.h"

#include "edge_detector.h"
#include "failsafe.h"
#include "motor_control.h"
#include "opponent_tracker.h"
#include "state_machine.h"

void robot_init(void)
{
    motor_control_init();
    edge_detector_init();
    opponent_tracker_init();
    failsafe_init();
    state_machine_init();
}

void robot_update(void)
{
    edge_detector_update();
    opponent_tracker_update();
    failsafe_update();
    state_machine_update();
    motor_control_update();
}

void robot_background(void)
{
    failsafe_background();
}
