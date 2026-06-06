# BSP

Board support files belong here.

Use this layer for direct STM32 HAL access, CubeMX handles, GPIO mappings, PWM
channels, ADC reads, UART writes, and timer setup helpers. Higher-level robot
modules should call BSP or custom driver functions instead of using HAL directly.

## What Goes Here

- GPIO helper functions.
- Timer/PWM helper functions.
- ADC read helpers.
- UART debug/telemetry write helpers.
- Mapping from logical robot names to CubeMX handles.

## Example

```c
void bsp_motor_left_set_pwm(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
}
```

Then the motor driver can call `bsp_motor_left_set_pwm()` without spreading
timer details through the rest of the robot code.

## Do Not Put Here

- Sumo strategy decisions.
- Sensor interpretation such as "edge detected".
- PID or motion behavior.
