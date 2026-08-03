# Motor Control Module (`motor.c`) Documentation

This document provides complete technical documentation for [`core/src/motor.c`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c). The module controls dual H-bridge DC motors via STM32 GPIO direction pins and Timer 1 PWM channels, featuring real-time battery voltage compensation, differential line tracking, non-blocking timing, and automated junction maneuvers.

---

## Table of Contents
1. [H-Bridge & PWM Hardware Control Flow](#h-bridge--pwm-hardware-control-flow)
2. [Battery Voltage Compensation Math](#battery-voltage-compensation-math)
3. [GPIO & Timer Pin Mapping](#gpio--timer-pin-mapping)
4. [Function Reference](#function-reference)
   - [`nonBlockingDelay`](#1-nonblockingdelay)
   - [`set_motor_speed`](#2-set_motor_speed)
   - [`follow_line`](#3-follow_line)
   - [`swing_turn_left`](#4-swing_turn_left)
   - [`swing_turn_right`](#5-swing_turn_right)
   - [`shoot_through`](#6-shoot_through)
   - [`handle_junction`](#7-handle_junction)
5. [Cross-Module Documentation Links](#cross-module-documentation-links)

---

## H-Bridge & PWM Hardware Control Flow

```mermaid
flowchart TD
    subgraph "Motor Speed Calculation"
        STEER[PID Steering Correction] --> DIFF[follow_line: Base Speed ± Correction]
        BATT[Battery Voltage Measurement] --> COMP[set_motor_speed Voltage Scaling]
        DIFF --> COMP
    end

    subgraph "Hardware Signal Output"
        COMP -->|Clamping -999 to 999| CLAMP[Speed Values]
        CLAMP -->|Direction Checks| DIR{Speed Sign}
        
        DIR -->|Speed > 0 (Forward)| FWD[Set IN1=1, IN2=0, TIM1 PWM = Speed]
        DIR -->|Speed < 0 (Backward)| REV[Set IN1=0, IN2=1, TIM1 PWM = |Speed|]
        DIR -->|Speed == 0 (Stop)| STOP[Set IN1=0, IN2=0, TIM1 PWM = 0]
        
        FWD --> HBRIDGE[Dual Channel H-Bridge Driver]
        REV --> HBRIDGE
        STOP --> HBRIDGE
        HBRIDGE --> MOTORS[DC Motors Left & Right]
    end
```

---

## Battery Voltage Compensation Math

> [!TIP]
> **Why Battery Voltage Compensation Matters**: As lithium-ion or LiPo battery packs discharge, supply voltage drops from ~12.4V down to 10V or lower. Without compensation, motor speeds degrade, throwing off calibrated PID gains.

The motor driver normalizes motor speeds relative to a fixed reference voltage (12.4V):

$$\text{Speed}_{\text{compensated}} = \text{constrain}\left( \text{Speed}_{\text{target}} \times \frac{12.4}{V_{\text{battery}}}, -999, 999 \right)$$

If the measured battery voltage drops below 1.0V (e.g., during initialization or sensor disconnect), a safe fallback nominal value of 12.4V is automatically used.

---

## GPIO & Timer Pin Mapping

The motor driver interfaces directly with GPIOB output pins and TIM1 PWM output channels:

| Motor Channel | Direction Pin 1 (Forward) | Direction Pin 2 (Backward) | PWM Speed Control Channel |
| :--- | :--- | :--- | :--- |
| **Left Motor** | `PB13` (GPIO Output) | `PB12` (GPIO Output) | `TIM1 Channel 1` (Period: 1000) |
| **Right Motor** | `PB14` (GPIO Output) | `PB15` (GPIO Output) | `TIM1 Channel 4` (Period: 1000) |

---

## Function Reference

### 1. `nonBlockingDelay`

- **Signature**: `uint8_t nonBlockingDelay(uint32_t ms)`
- **Location**: [`motor.c: L14-L29`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c#L14-L29)
- **Parameters**: `ms` - Delay duration in milliseconds.
- **Returns**: `uint8_t` - `1` while delay is active, `0` when delay has completed.
- **Description**: Non-blocking delay function using `HAL_GetTick()`. Maintains static state variables so execution loops do not stall CPU instruction cycles during maneuver timing.

---

### 2. `set_motor_speed`

- **Signature**: `void set_motor_speed(int left_motor, int right_motor, float battery_voltage)`
- **Location**: [`motor.c: L31-L84`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c#L31-L84)
- **Parameters**: 
  - `left_motor`: Desired left motor speed (`-999` to `999`).
  - `right_motor`: Desired right motor speed (`-999` to `999`).
  - `battery_voltage`: Measured battery voltage in Volts.
- **Description**:
  Applies voltage compensation, clamps speeds to `[-999, 999]`, configures H-bridge GPIO direction pins (`PB12-PB15`), and sets TIM1 Compare PWM registers (`TIM1->CCR1`, `TIM1->CCR4`).

---

### 3. `follow_line`

- **Signature**: `void follow_line(int correction, Sensor_Array *sensor_array)`
- **Location**: [`motor.c: L86-L92`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c#L86-L92)
- **Parameters**: 
  - `correction`: Calculated PID correction value.
  - `sensor_array`: Pointer to `Sensor_Array` instance containing `base_speed`.
- **Description**:
  Applies differential steering:
  $$\text{Left Speed} = \text{Base Speed} + \text{Correction}$$
  $$\text{Right Speed} = \text{Base Speed} - \text{Correction}$$
  Calls `set_motor_speed` with current battery voltage read from `dma_buffer`.

---

### 4. `swing_turn_left`

- **Signature**: `void swing_turn_left(Sensor_Array *sa, int speed)`
- **Location**: [`motor.c: L94-L121`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c#L94-L121)
- **Description**: Executes a sharp left swing turn:
  1. Drives forward for 100 ms.
  2. Rotates left (left motor `-speed`, right motor `speed`) for 100 ms.
  3. Continuously samples sensors until center sensors (index 3 or 4) re-acquire the line (`sa->array[3].on == 1 || sa->array[4].on == 1`).
  4. Applies brief reverse braking pulse for 20 ms and stops.

---

### 5. `swing_turn_right`

- **Signature**: `void swing_turn_right(Sensor_Array *sa, int speed)`
- **Location**: [`motor.c: L123-L149`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c#L123-L149)
- **Description**: Executes a sharp right swing turn:
  1. Drives forward for 100 ms.
  2. Rotates right (left motor `speed`, right motor `-speed`) for 100 ms.
  3. Continuously samples sensors until center sensors (index 2 or 3) re-acquire the line (`sa->array[2].on == 1 || sa->array[3].on == 1`).
  4. Applies brief reverse braking pulse for 20 ms and stops.

---

### 6. `shoot_through`

- **Signature**: `void shoot_through(Sensor_Array *sa, int speed)`
- **Location**: [`motor.c: L151-L171`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c#L151-L171)
- **Description**: Drives straight across T-junctions or cross intersections at specified `speed` for 100 ms, then halts motors.

---

### 7. `handle_junction`

- **Signature**: `void handle_junction(Sensor_Array *sa, JunctionType j, int speed)`
- **Location**: [`motor.c: L173-L197`](file:///H:/Jaishnu/stm_workspace_2/line_follower/core/src/motor.c#L173-L197)
- **Description**: Junction maneuver dispatcher. Invokes `swing_turn_left`, `swing_turn_right`, or `shoot_through` depending on classified `JunctionType` enum.

---

## Cross-Module Documentation Links

- 📡 [Main Control Loop & System Architecture (`docs/main.md`)](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/main.md)
- 📊 [Sensor Module Architecture (`docs/sensor_module.md`)](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/sensor_module.md)
- 🖥️ [Python PID Tuner Dashboard (`docs/pid_tuner.md`)](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/pid_tuner.md)
- 📶 [Bluetooth Module Documentation (`docs/bluetooth.md`)](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/bluetooth.md)
- 🛠️ [Utility Helpers Documentation (`docs/utils.md`)](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/utils.md)
- 📑 [Documentation Index (`docs/README.md`)](file:///H:/Jaishnu/stm_workspace_2/line_follower/docs/README.md)
