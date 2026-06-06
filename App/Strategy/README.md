# Strategy

This folder decides what the robot should do during a match.

## What Goes Here

- Sumo state machine.
- Search behavior.
- Attack behavior.
- Edge escape behavior.
- Recovery and fault behavior.

## Example

The strategy reads service-level information and chooses a state:

```c
if (edge_detector_is_edge_detected()) {
    current_state = ROBOT_STATE_EDGE_ESCAPE;
} else if (opponent_tracker_has_target()) {
    current_state = ROBOT_STATE_ATTACK;
} else {
    current_state = ROBOT_STATE_SEARCH;
}
```

Then it asks the control layer for motion:

```c
motor_control_set_command(motion_forward(700));
```

## Do Not Put Here

- Raw GPIO, ADC, or timer code.
- Sensor calibration math that belongs in drivers/services.
- Large blocks of duplicated motor code.

Strategy should be readable like the robot's match plan.
