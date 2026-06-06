# App

This folder contains the robot-owned firmware. STM32CubeMX can regenerate
`Core/` and `Drivers/` at the project root, but this `App/` folder should stay
under our control.

## Main Idea

CubeMX initializes the STM32. After that, it calls `app_main()`.

```c
MX_GPIO_Init();
MX_I2C1_Init();
MX_TIM2_Init();
MX_TIM3_Init();

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

- `Inc/`: all robot-owned headers.
- `Src/`: all robot-owned source files.

## Rule of Thumb

Keep STM32CubeMX-generated code in the project-root `Core/` and `Drivers/`
folders. Keep robot behavior, drivers, services, and control code in `App/Src`
with matching public headers in `App/Inc`.
