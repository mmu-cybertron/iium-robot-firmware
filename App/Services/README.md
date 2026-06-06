# Services

This folder turns raw driver data into useful robot information.

## What Goes Here

- Edge detection.
- Opponent tracking.
- Battery/failsafe checks.
- Telemetry and diagnostics later.
- Combining multiple sensors into one clear status.

## Example

The line sensor driver reports individual sensor status. The service answers a
simple question for the strategy:

```c
uint8_t edge_detector_is_edge_detected(void)
{
    return current_edges.front_left ||
           current_edges.front_right ||
           current_edges.rear_left ||
           current_edges.rear_right;
}
```

## Do Not Put Here

- Direct STM32 HAL calls.
- PWM timer setup.
- Attack/search/escape strategy.

Services should describe the robot's situation, not choose the full behavior.
