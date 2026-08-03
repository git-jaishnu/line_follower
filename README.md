# ⚡ High-Speed STM32 Line Follower Robot with Real-Time Python PID Tuner

[![STM32 Architecture](https://img.shields.io/badge/Microcontroller-STM32F4-00599C?logo=stmicroelectronics&logoColor=white)](https://www.st.com/)
[![Language](https://img.shields.io/badge/Language-C%20%2F%20Python-00599C?logo=c&logoColor=white)](https://en.wikipedia.org/wiki/C_(programming_language))
[![GUI Dashboard](https://img.shields.io/badge/GUI-Tkinter%20%2B%20Matplotlib-ff69b4?logo=python&logoColor=white)](pid_tuner.py)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An advanced, high-speed line follower robot platform engineered on the **STM32F4** ARM Cortex-M4 microcontroller. This system features **zero-CPU-overhead continuous ADC sampling via DMA**, **closed-loop PID steering control with dynamic time-delta computation**, **real-time battery voltage compensation**, and a **hardware-agnostic, modular sensor processing module**. 

Included in this repository is a custom desktop GUI application ([`pid_tuner.py`](pid_tuner.py)) that streams real-time telemetry over Bluetooth/UART, plots line error, correction outputs, and motor speeds at ~60ms intervals, and allows live gain tuning without reflashing firmware.

---

## 📑 Detailed Module Documentation Links

- 📊 [**Modular Sensor Module Architecture (`docs/sensor_module.md`)**](docs/sensor_module.md)
- 📡 [**Main System Architecture & Control Loop Logic (`docs/main.md`)**](docs/main.md)
- ⚡ [**Motor Control & Battery Voltage Compensation (`docs/motor.md`)**](docs/motor.md)
- 🖥️ [**Python PID Tuner Dashboard Guide & Protocol (`docs/pid_tuner.md`)**](docs/pid_tuner.md)
- 📶 [**Bluetooth Communication Module (`docs/bluetooth.md`)**](docs/bluetooth.md)
- 🛠️ [**Utility Helpers & Voltage Conversion Math (`docs/utils.md`)**](docs/utils.md)
- 📚 [**Documentation Index (`docs/README.md`)**](docs/README.md)



---

## 🌟 Key Features & Highlights

- **⚡ Zero-CPU-Overhead Sensor Acquisition**: Uses STM32 ADC1 in continuous multi-channel scan mode paired with DMA2 Stream 0 to transfer 12-bit sensor data into RAM asynchronously.
- **🧱 Modular & Hardware-Agnostic Sensor Architecture**: The core sensor module ([`core/src/sensor_module.c`](core/src/sensor_module.c)) decouples physical array hardware from navigation math. Supports any IR array size (4, 6, 8, 12, 16 sensors), custom positional weights, dynamic per-sensor min/max auto-calibration, and analog or digital line-error algorithms.
- **🔋 Battery Voltage Compensation**: Continuously measures battery voltage on a dedicated ADC channel and dynamically scales motor PWM duty cycles to maintain uniform speed as the battery depletes.
- **🧭 Automatic Junction Detection & Maneuvers**: Pattern-matching classifiers detect T-junctions, 90° left/right turns, and cross intersections, executing specialized turn routines.
- **📊 Real-Time Python PID Dashboard**: Custom Tkinter/Matplotlib GUI ([`pid_tuner.py`](pid_tuner.py)) featuring dual Y-axis plots, live 12-bit IR bar charts, CSV recording/exporting, debug packet logging, and a built-in step-by-step PID tuning guide.

---

## 🏗️ System Architecture

```mermaid
flowchart TD
    subgraph "Hardware Layer"
        IR["IR Reflectance Sensor Bar (4, 6, 8, or 16 Channels)"] -->|Analog Voltage| ADC[STM32 ADC1 Peripheral]
        BATT[Battery Voltage Divider] -->|Sense Channel| ADC
        ADC -->|Zero-CPU DMA Transfer| DMA[DMA2 Stream 0]
        DMA -->|Circular Buffer| DMABUF[dma_buffer in RAM]
        BTN[Push Button PC14] -->|EXTI Interrupt| EXTI[EXTI ISR]
        BT_HW[Bluetooth Transceiver HC-05/06] -->|USART1 RX Interrupt| UART_ISR[HAL_UART_RxCpltCallback]
    end

    subgraph "Firmware Engine (core/src)"
        DMABUF -->|Sync & Re-index| SYNC[Sync_Sensors]
        SYNC -->|Filter & Clamping| PROC[processSensors & binarizeSensors]
        PROC --> JUNC{Junction Detected?}
        
        JUNC -->|Yes| MANEUVER[handle_junction]
        MANEUVER --> TURN[Swing Turn / Shoot Through]
        
        JUNC -->|No| CALC[get_line_error_digital]
        CALC --> PID[calculate_pid]
        PID --> FOLLOW[follow_line]
    end

    subgraph "Ground Station & Telemetry"
        UART_ISR -->|Parse Commands| PARSER[processBluetoothCommand]
        PARSER -->|Update Gains / Speeds| PID
        TELEM[Send_Telemetry] -->|USART1 TX| BT_HW
        BT_HW -->|Wireless Telemetry Stream| DASH["Python PID Tuner GUI (pid_tuner.py)"]
    end

    subgraph "Actuators"
        TURN --> PWM[TIM1 PWM CH1 & CH4 + GPIOB Pins]
        FOLLOW --> PWM
        PWM --> MOTORS[Dual H-Bridge DC Motors]
    end
```

---

## 🧩 Modular Sensor Module Architecture (`sensor_module.c`)

> [!NOTE]
> 📖 **Full Dedicated Documentation**: For a complete function-by-function reference, pipeline breakdown, and mathematical derivations, see the standalone [**Sensor Module Architecture Guide (`docs/sensor_module.md`)**](docs/sensor_module.md).

One of the standout features of this project is its **completely modular and hardware-decoupled sensor subsystem** implemented in [`core/src/sensor_module.c`](core/src/sensor_module.c) and [`core/inc/types.h`](core/inc/types.h).

> [!IMPORTANT]
> Unlike standard line follower codebases that hardcode sensor pin counts or specific sensor models, this architecture abstracts sensor hardware into dynamic data structures.

```mermaid
flowchart LR
    subgraph "Hardware Agnostic Layer"
        S1[Sensor Struct 0]
        S2[Sensor Struct 1]
        SN[Sensor Struct N-1]
    end

    subgraph "Sensor_Array Container Struct"
        ARRAY_PTR[*array Pointer] --> S1 & S2 & SN
        WEIGHTS_PTR[*weights Pointer] --> W[Custom Weights Array]
        COUNT[number_of_sensors = N]
    end

    subgraph "Dynamic Processing & Calibration"
        S1 & S2 & SN --> CALIB["autoCalibrate (Independent adc_min / adc_max per sensor)"]
        CALIB --> NORM["processSensors (Dynamic Range Scaling & Clamping)"]
        NORM --> MATH{Choose Algorithm}
        MATH -->|Analog COG| ANA["get_line_error (Sub-millimeter Continuous Error)"]
        MATH -->|Digital Active| DIG["get_line_error_digital (High-Speed Discrete Error)"]
    end
```

### Why This Design is Superior:

1. **Any Array Size ($N$ Sensors)**:
   Change `number_of_sensors` in [`main.c`](core/src/main.c) or runtime config. Supports 4, 6, 8, 12, or 16-channel sensor bars (e.g., Pololu QTR-8A, QTR-8D, TCRT5000 arrays, or custom phototransistor boards) without modifying calculation code.
2. **Custom Positional Weighting**:
   Assign any integer weight array (e.g., `{-4, -3, -2, -1, 1, 2, 3, 4}` or non-linear `{-8, -4, -2, -1, 1, 2, 4, 8}`) to match physical sensor geometry.
3. **Per-Sensor Independent Auto-Calibration**:
   [`autoCalibrate`](docs/sensor_module.md#3-autocalibrate) spins the robot over the track to record independent `adc_min` and `adc_max` bounds for each individual sensor, compensating for manufacturing variations and height offsets.
4. **Dual Line Position Algorithms**:
   - **Continuous Analog Mode** ([`get_line_error`](docs/sensor_module.md#6-get_line_error)):
     $$\text{Position Error} = \frac{\sum (v_i \times w_i)}{\sum v_i}$$
     *(where $v_i$ is mapped analog intensity and $w_i$ is sensor weight)*
   - **Discrete Digital Mode** ([`get_line_error_digital`](docs/sensor_module.md#7-get_line_error_digital)):
     $$\text{Position Error} = \frac{\sum_{\text{active}} w_i}{N_{\text{active}}}$$
     *(where $w_i$ is active sensor weight and $N_{\text{active}}$ is total active sensors)*

👉 *Read the full standalone [**Sensor Module Documentation (`docs/sensor_module.md`)**](docs/sensor_module.md) for deep-dive details.*

---

## 📖 Complete Replication & User Guide

Follow this guide to build, wire, flash, and tune your own STM32 line follower robot.

### 1. Hardware Requirements

| Component | Recommended Part | Function |
| :--- | :--- | :--- |
| **Microcontroller** | STM32F401RE / STM32F411CE Black Pill / Nucleo | Main processing MCU |
| **IR Sensor Bar** | Pololu QTR-8A / TCRT5000 8-Channel Array | Line detection (Connected to ADC1 CH0–CH7) |
| **Motor Driver** | TB6612FNG / L298N Dual H-Bridge | Motor power control (PWM on TIM1 CH1 & CH4) |
| **DC Motors** | N20 12V 600RPM Micro Gear Motors + Wheels | Drive actuators |
| **Bluetooth Module** | HC-05 / HC-06 Transceiver | Wireless telemetry & tuning (USART1 9600 baud) |
| **Battery & Voltage Divider** | 3S LiPo (11.1V–12.6V) + 10kΩ / 3.3kΩ Divider | Power supply & battery voltage sensing (ADC1 CH9) |
| **Push Button** | Momentary Push Button | User start/stop trigger (Connected to `PC14` EXTI) |

---

### 2. Wiring & Pinout Table

| STM32 Pin | Peripheral Function | Connected Hardware |
| :--- | :--- | :--- |
| `PA0` – `PA7` | `ADC1_IN0` – `ADC1_IN7` | IR Sensors 1 to 8 (ADC DMA) |
| `PB1` | `ADC1_IN9` | Battery Voltage Sensing (Resistor Divider) |
| `PB12` | `GPIO_Output` | Left Motor Backward (IN2) |
| `PB13` | `GPIO_Output` | Left Motor Forward (IN1) |
| `PB14` | `GPIO_Output` | Right Motor Forward (IN3) |
| `PB15` | `GPIO_Output` | Right Motor Backward (IN4) |
| `PA8` | `TIM1_CH1` (PWM Output) | Left Motor Speed PWM |
| `PA11` | `TIM1_CH4` (PWM Output) | Right Motor Speed PWM |
| `PA9` / `PA10` | `USART1_TX` / `USART1_RX` | Bluetooth HC-05/06 TX/RX |
| `PC14` | `GPIO_EXTI14` (Falling Edge) | User Start/Stop Button |

---

### 3. Software Environment & Toolchain

1. **Firmware Toolchain**:
   - [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) or VS Code with STM32 Extension + `arm-none-eabi-gcc`.
2. **Dashboard Toolchain**:
   - Python 3.8+
   - Install dependencies:
     ```bash
     pip install pyserial matplotlib
     ```

---

### 4. Flashing Firmware & Running Dashboard

1. Open the project workspace in STM32CubeIDE.
2. Build the project (`Ctrl+B`) and flash the `.elf` / `.bin` binary to the STM32 board via ST-Link.
3. Power on the robot and pair your computer with the Bluetooth module.
4. Launch the PID Tuner Dashboard:
   ```bash
   python pid_tuner.py
   ```
5. Select your Bluetooth COM port, click **Connect**, and observe real-time plots.

---

### 5. Step-by-Step PID Tuning Guide

> [!TIP]
> Use the built-in guide window in `pid_tuner.py` by clicking **`📖 PID Tuning Guide`**.

```
STEP 0: Reset Gains         --> Set Kp = 0.5, Ki = 0.0, Kd = 0.0. Click Send PID.
STEP 1: Tune Proportional   --> Increase Kp in steps of 0.5 until robot follows line 
                                with oscillation, then back off ~20%.
STEP 2: Tune Derivative     --> Increase Kd in steps of 0.2 to damp out oscillation.
STEP 3: Tune Integral       --> Keep Ki = 0.0 unless persistent position offset exists.
STEP 4: Adjust Speed        --> Increase Base Speed (BS) & set PID Limit ≈ Base Speed × 0.6.
```

---

## 📁 Repository Structure & Module Documentation

Below is the complete project sitemap. Click any link to access detailed module documentation:

```text
line_follower/
├── README.md                          <-- You are here (Repository Guide)
├── pid_tuner.py                       <-- Real-time Python GUI Tuning Dashboard
├── docs/                              <-- Technical Documentation Folder
│   ├── README.md                      <-- Documentation Index
│   ├── main.md                        <-- System Architecture & Main Loop Logic
│   ├── sensor_module.md               <-- Modular Sensor Architecture & Math
│   ├── motor.md                       <-- PWM, H-Bridge & Voltage Compensation
│   ├── bluetooth.md                   <-- Wireless Serial Protocol & Command Parser
│   ├── pid_tuner.md                   <-- Dashboard User Guide & Interactive Demo
│   └── utils.md                       <-- Math Clamping & Voltage Scaling Math
└── core/                              <-- STM32 Embedded Firmware Source
    ├── inc/
    │   ├── bluetooth.h
    │   ├── main.h
    │   ├── motor.h
    │   ├── sensor_module.h
    │   ├── types.h                    <-- Core Structures (Sensor, Sensor_Array, PID)
    │   └── utils.h
    └── src/
        ├── bluetooth.c                <-- Bluetooth UART Handler
        ├── main.c                     <-- Application Entry Point & Control Loop
        ├── motor.c                    <-- Motor Driver & Junction Maneuvers
        ├── sensor_module.c            <-- Modular Sensor Engine & PID Calculation
        └── utils.c                    <-- Utility Conversion Functions
```

---


### Future Improvements:

1. **Lighter Body**: Heavy body due to it being PCB created a speed bottleneck.
2. **Smaller Battery**: High mAh not required for competitions.
3. **Suction Technology** : Using a fan to increase downforce.
4. **Better Wheels** : To improve turning.
5. **Better Isolation/Protection** : Prevent component damage.
6. **Better Array Design**

---



## 📜 License

This project is open-source under the [MIT License](LICENSE).
