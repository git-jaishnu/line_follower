# Project Documentation Index

This directory contains comprehensive architectural documentation for the STM32 line follower robot software stack and Python tuning tools.

## Documentation Files

| File / Tool | Documentation File | Primary Responsibility |
| :--- | :--- | :--- |
| [`pid_tuner.py`](file:///H:/Jaishnu/stm_workspace_2/line_follower/pid_tuner.py) | [`docs/pid_tuner.md`](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/pid_tuner.md) | Desktop GUI dashboard for real-time telemetry plotting, Bluetooth PID parameter tuning, CSV recording, and debug monitoring. |
| [`core/src/main.c`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/main.c) | [`docs/main.md`](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/main.md) | Core control loop logic, DMA/ADC scanning architecture, EXTI button interrupts, UART telemetry, and junction maneuvers. |
| [`core/src/sensor_module.c`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/sensor_module.c) | [`docs/sensor_module.md`](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/sensor_module.md) | ADC DMA array synchronization, calibration, normalization, error calculations, PID, and junction detection. |
| [`core/src/bluetooth.c`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/bluetooth.c) | [`docs/bluetooth.md`](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/bluetooth.md) | Transmitting UART messages over Bluetooth and parsing dynamic runtime tuning commands. |
| [`core/src/motor.c`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c) | [`docs/motor.md`](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/motor.md) | H-bridge GPIO control, PWM speed control, voltage compensation, line tracking, and junction maneuvers. |
| [`core/src/utils.c`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/utils.c) | [`docs/utils.md`](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/utils.md) | Mathematical clamping helpers (`constrain_int`, `constrain_float`) and battery voltage ADC conversions. |
