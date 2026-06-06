# Firmware Structure

This project keeps STM32CubeMX-generated code separate from robot-owned
firmware.

## Ownership

- `Core/` and `Drivers/`: generated STM32 startup, peripheral initialization,
  HAL, and CMSIS code.
- `App/Inc/`: robot-owned headers.
- `App/Src/`: robot-owned source files, including lifecycle, hardware drivers,
  services, motion control, and strategy.

## App Layout

```text
App/
|-- Inc/
|   |-- app_main.h
|   |-- robot.h
|   |-- robot_config.h
|   |-- robot_types.h
|   |-- motor_driver.h
|   |-- motor_control.h
|   |-- motion.h
|   |-- line_sensor.h
|   |-- distance_sensor.h
|   |-- edge_detector.h
|   |-- opponent_tracker.h
|   |-- failsafe.h
|   `-- state_machine.h
|-- Src/
|   |-- app_main.c
|   |-- robot.c
|   |-- motor_driver.c
|   |-- motor_control.c
|   |-- motion.c
|   |-- line_sensor.c
|   |-- distance_sensor.c
|   |-- edge_detector.c
|   |-- opponent_tracker.c
|   |-- failsafe.c
|   `-- state_machine.c
`-- README.md
```

## CubeMX Integration

Add this include to CubeMX `Core/Src/main.c`:

```c
#include "app_main.h"
```

Call `app_main()` after all `MX_*_Init()` functions:

```c
MX_GPIO_Init();
MX_I2C1_Init();
MX_TIM2_Init();
MX_TIM3_Init();

app_main();
```

Add this include path in STM32CubeIDE:

```text
App/Inc
```
