# Control

This folder converts desired movement into motor commands.

## What Goes Here

- Motor command limiting.
- Forward/reverse/rotate helper functions.
- PID control.
- Acceleration ramping.
- Left/right motor mixing.

## Example

The strategy can ask for a simple motion:

```c
motor_control_set_command(motion_forward(700));
```

The control layer turns that into a left/right command:

```c
motor_command_t motion_forward(int16_t pwm)
{
    motor_command_t command = { pwm, pwm };
    return command;
}
```

## Do Not Put Here

- Direct HAL calls.
- Raw ADC reading.
- Decisions such as "attack" or "escape".

Those belong in `BSP/`, `Drivers/`, and `Strategy/`.
