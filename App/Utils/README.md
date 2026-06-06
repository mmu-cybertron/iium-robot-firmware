# Utils

Small reusable helpers belong here, such as filters, ring buffers, math helpers,
and time utilities.

## What Goes Here

- Moving average filters.
- Low-pass filters.
- Math clamp/map helpers.
- Ring buffers.
- Time helper functions.

## Example

```c
int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}
```

## Do Not Put Here

- Robot strategy.
- Hardware-specific HAL code.
- One-off functions used by only one file.

Only put code here when it is small, reusable, and independent from the robot's
hardware choices.
