# App

This folder contains the robot-owned firmware. STM32CubeMX can regenerate
`Core/` and `Drivers/` at the project root, but this `App/` folder should stay
under our control.

## Main Idea

CubeMX initializes the STM32. After that, it calls `app_main()`.

```c
MX_GPIO_Init();
MX_ADC1_Init();
MX_TIM1_Init();

app_main();
```

Inside `app_main()`, the robot runs a fixed update loop:

```c
while (1) {
    robot_update();
    robot_background();
}
```

## Folder Map

- `Core/`: robot lifecycle, config, shared types.
- `BSP/`: STM32 HAL access and board pin/timer/ADC mapping.
- `Drivers/`: specific hardware drivers, such as motors and sensors.
- `Services/`: meaningful robot information built from drivers.
- `Control/`: motion and motor command logic.
- `Strategy/`: sumo behavior and state machine.
- `Utils/`: small reusable helper code.

## Rule of Thumb

If the code knows about STM32 pins, timers, ADC, or UART, put it near `BSP/` or
`Drivers/`. If the code decides how the robot should behave in the ring, put it
near `Strategy/`.
