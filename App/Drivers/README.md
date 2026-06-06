# Drivers

This folder contains drivers for specific hardware parts on the robot.

## What Goes Here

- Motor driver code.
- Line sensor code.
- Distance sensor code.
- Battery monitor code.
- Encoder or IMU code later.

## Example

A line sensor driver should return raw, simple information:

```c
edge_status_t line_sensor_read_edges(void)
{
    edge_status_t status = { 0, 1, 0, 0 };
    return status;
}
```

Then `Services/edge_detector.c` can decide what that means for the robot.

## Do Not Put Here

- Full robot behavior.
- State machine logic.
- High-level meanings such as "escape now".

Drivers should know hardware. Services and Strategy should know behavior.
