# Core

This folder contains the robot application core: startup flow, shared
configuration, shared types, and the robot lifecycle.

## What Goes Here

- `robot.c`: calls init/update functions for the whole robot.
- `robot_config.h`: constants such as update rate, PWM limits, and thresholds.
- `robot_types.h`: shared enums and structs.
- `robot_pins.h`: logical pin names used by the robot code.

## Example

Use `robot_update()` as the one place that coordinates the main robot cycle:

```c
void robot_update(void)
{
    edge_detector_update();
    opponent_tracker_update();
    failsafe_update();
    state_machine_update();
    motor_control_update();
}
```

## Do Not Put Here

- Direct HAL pin writes.
- Motor driver details.
- Long strategy decisions.

Those belong in `BSP/`, `Drivers/`, or `Strategy/`.
