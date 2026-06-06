# iium-robot-firmware

STM32 firmware structure for a 3kg sumo robot.

The project separates STM32CubeMX-generated code from robot-owned firmware:

- `App/app_main.c`: application entrypoint called from CubeMX `main.c`.
- `App/Core/`: robot lifecycle, configuration, shared types.
- `App/BSP/`: board support and direct STM32 HAL access.
- `App/Drivers/`: motor, sensor, battery, encoder, and device drivers.
- `App/Services/`: edge detection, opponent tracking, failsafe, telemetry.
- `App/Control/`: motor control and motion helpers.
- `App/Strategy/`: sumo state machine and behavior.
- `App/Utils/`: small reusable helpers.

See `docs/firmware-structure.md` for CubeMX integration notes.
