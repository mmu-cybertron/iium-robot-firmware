# iium-robot-firmware

STM32 firmware structure for a 3kg sumo robot.

The project separates STM32CubeMX-generated code from robot-owned firmware:

- `App/Inc/`: robot-owned headers.
- `App/Src/`: robot-owned source files, including `app_main.c`.
- `Core/`: STM32CubeMX-generated startup and peripheral initialization.
- `Drivers/`: STM32 HAL and CMSIS driver code.

See `docs/firmware-structure.md` for CubeMX integration notes.
