# `sensor_module` — API Documentation

> **Author:** jaish | **Created:** Apr 11, 2026 | **Target:** STM32F4xx (HAL)

---

## Table of Contents

1. [Data Structures](#data-structures)
2. [Global Variables](#global-variables)
3. [Function Reference](#function-reference)
4. [Typical Usage Flow](#typical-usage-flow)

---

## Data Structures

### `Sensor`

Represents a single IR/analog sensor on the array.

```c
typedef struct {
    uint16_t adc_raw;       // Raw ADC reading from DMA buffer (0–4095)
    uint16_t adc_max;       // Calibrated maximum ADC value
    uint16_t adc_min;       // Calibrated minimum ADC value
    uint16_t weight;        // Positional weight used in line-error calculation
    int      threshold;     // Binary on/off threshold (mapped_value scale: 0–1000)
    int      mapped_value;  // Normalized & inverted value (0 = off line, 1000 = on line)
    int      on;            // 1 if sensor is over the line, 0 otherwise
} Sensor;
```

| Field | Range | Description |
|-------|-------|-------------|
| `adc_raw` | 0–4095 | Direct ADC reading, refreshed each DMA cycle |
| `adc_min` / `adc_max` | 0–4095 | Set during calibration; define the white/black extremes |
| `weight` | user-defined | Signed positional weight for weighted error (e.g. −3500 to +3500) |
| `threshold` | 0–1000 | `mapped_value` above this is treated as "on the line" |
| `mapped_value` | 0–1000 | **1000** = fully on line, **0** = fully off line |
| `on` | 0 or 1 | Set by `binarizeSensors()` |

---

### `Sensor_Array`

Container that holds all sensors and their configuration.

```c
typedef struct {
    int      number_of_sensors; // Total sensor count (typically 8)
    Sensor  *array;             // Pointer to array of Sensor structs
    int     *weights;           // Pointer to positional weight array
} Sensor_Array;
```

**Setup example:**

```c
#define NUM_SENSORS 8

Sensor  sensors[NUM_SENSORS];
int     weights[NUM_SENSORS] = { -3500, -2500, -1500, -500, 500, 1500, 2500, 3500 };

Sensor_Array sensor_array = {
    .number_of_sensors = NUM_SENSORS,
    .array             = sensors,
    .weights           = weights
};
```

> Weights run left-to-right. Negative = left of center, positive = right of center.  
> Adjust magnitudes to tune how aggressively the error responds to off-center deviation.

---

### `PID_Controller`

Holds PID gain constants and running state.

```c
typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float integral;     // Accumulated integral — do NOT modify at runtime
    float last_error;   // Previous error value — do NOT modify at runtime
    float limit;        // Output saturation limit (also used as integral windup cap)
} PID_Controller;
```

**Initialization example:**

```c
PID_Controller pid = {
    .Kp         = 0.8f,
    .Ki         = 0.0f,
    .Kd         = 5.0f,
    .integral   = 0.0f,   // must be 0 at start
    .last_error = 0.0f,   // must be 0 at start
    .limit      = 200.0f  // correction clamped to ±200
};
```

---

## Global Variables

```c
volatile uint16_t dma_buffer[NUM_SENSORS];
```

Populated automatically by DMA from the ADC peripheral. The index-to-sensor mapping is **not 1-to-1** — see [`Sync_Sensors`](#sync_sensors) for the remapping table. Never write to this buffer manually.

---

## Function Reference

---

### `Initialize_Sensor_Array`

```c
void Initialize_Sensor_Array(Sensor_Array *sensor_array);
```

Seeds each `Sensor` in the array with default values before use.

**What it sets per sensor:**

| Field | Value |
|-------|-------|
| `weight` | copied from `sensor_array->weights[i]` |
| `adc_min` | `0` |
| `adc_max` | `4095` |
| `threshold` | `SENSOR_THRESHOLD` (compile-time constant) |

Call **once** after declaring your `Sensor_Array`, before the main loop.

```c
Initialize_Sensor_Array(&sensor_array);
```

---

### `Sync_Sensors`

```c
void Sync_Sensors(Sensor_Array *sensor_array);
```

Copies values from `dma_buffer[]` into `Sensor.adc_raw` for each sensor, applying a hardware channel remapping to account for the physical PCB wiring order.

**Channel remapping:**

| Logical sensor index | DMA buffer index |
|:---:|:---:|
| 0 | 4 |
| 1 | 3 |
| 2 | 5 |
| 3 | 2 |
| 4 | 6 |
| 5 | 1 |
| 6 | 7 |
| 7 | 0 |

Call this **at the top of every control loop iteration**, before any processing.

```c
Sync_Sensors(&sensor_array);
```

---

### `autoCalibrate`

```c
void autoCalibrate(Sensor_Array *sensor_array, uint32_t duration_ms, int speed);
```

Performs an automatic min/max calibration sweep. The robot spins in place while recording the extreme ADC values seen by each sensor, establishing the white/black baseline for normalization.

| Parameter | Type | Description |
|-----------|------|-------------|
| `sensor_array` | `Sensor_Array*` | Target sensor array |
| `duration_ms` | `uint32_t` | Sweep duration in milliseconds (e.g. `2000`) |
| `speed` | `int` | Motor speed magnitude; robot turns at `(−speed, +speed)` |

**Sequence:**
1. Resets `adc_min = 0`, `adc_max = 4096` for all sensors.
2. Starts motors spinning: `set_motor_speed(-speed, speed)`.
3. Polls `adc_raw` every 1 ms, recording true min/max seen over `duration_ms`.
4. Stops motors: `set_motor_speed(0, 0)`.

> **Important:** `Sync_Sensors()` is **not** called inside `autoCalibrate()`. The DMA buffer must be refreshed externally (e.g. via ADC conversion complete interrupt/callback) during the sweep for calibration to capture accurate readings.

```c
autoCalibrate(&sensor_array, 2000, 40);
```

---

### `processSensors`

```c
void processSensors(Sensor_Array *sensor_array);
```

Normalizes each sensor's `adc_raw` against its calibrated range and stores the result in `mapped_value` on a 0–1000 scale, **inverted** so that darker surface = higher value.

**Formula per sensor:**

```
range        = adc_max - adc_min   (floored to 1 to avoid divide-by-zero)
norm         = (adc_raw - adc_min) * 1000 / range   (clamped 0–1000)
mapped_value = 1000 - norm
```

| `mapped_value` | Surface |
|:-:|---|
| `1000` | Fully over dark/black line |
| `0` | Fully over bright/white surface |

Call **after** `Sync_Sensors()` and **before** `binarizeSensors()` or `get_line_error()`.

```c
Sync_Sensors(&sensor_array);
processSensors(&sensor_array);
```

---

### `binarizeSensors`

```c
void binarizeSensors(Sensor_Array *sensor_array);
```

Sets each `Sensor.on` flag by comparing `mapped_value` against `threshold`.

```
mapped_value > threshold  →  on = 1
mapped_value ≤ threshold  →  on = 0
```

Call **after** `processSensors()`. The `.on` flags are consumed by `detect_left()` and `detect_right()`.

```c
processSensors(&sensor_array);
binarizeSensors(&sensor_array);
```

---

### `get_line_error`

```c
int get_line_error(Sensor_Array *sensor_array);
```

Computes a weighted position error by summing each sensor's `mapped_value` multiplied by its `weight`.

**Formula:**

```
weighted_sum = Σ (mapped_value[i] × weight[i])   for i = 0..7
return weighted_sum
```

| Return value | Meaning |
|:---:|---|
| `0` | Line is centered |
| Large negative | Line is to the **left** |
| Large positive | Line is to the **right** |

Feed the return value directly into `calculate_pid()` as the `error` argument.

```c
int error      = get_line_error(&sensor_array);
int correction = calculate_pid(&pid, error, dt);
```

---

### `calculate_pid`

```c
int calculate_pid(PID_Controller *pid, int error, float dt);
```

Standard PID controller with **integral windup guard** and **output saturation**. Updates internal state and returns an integer correction value.

| Parameter | Type | Description |
|-----------|------|-------------|
| `pid` | `PID_Controller*` | PID instance with gains and state |
| `error` | `int` | Current error (typically from `get_line_error()`) |
| `dt` | `float` | Time delta in seconds since last call |

**Computation:**

```
P        = Kp × error
integral += error × dt            (clamped to ±limit)
I        = Ki × integral
D        = Kd × (error − last_error) / dt
output   = P + I + D              (clamped to ±limit, returned as int)
```

```c
float dt       = 0.005f; // 5 ms loop
int correction = calculate_pid(&pid, error, dt);

set_motor_speed(BASE_SPEED - correction, BASE_SPEED + correction);
```

---

### `count_active_sensors`

```c
int count_active_sensors(Sensor_Array *sensor_array);
```

Returns the number of sensors whose `mapped_value` exceeds `threshold`. Useful for detecting junctions, intersections, or a fully-lost line state.

| Return value | Likely scenario |
|:---:|---|
| `0` | Line completely lost |
| `1`–`2` | Normal line following |
| `5`–`8` | Junction / T-intersection / end of track |

> Does **not** depend on the `.on` flag — can be called before or independently of `binarizeSensors()`.

```c
int active = count_active_sensors(&sensor_array);
if (active == 0) {
    set_motor_speed(0, 0); // line lost — stop or recover
}
```

---

### `detect_left`

```c
int detect_left(Sensor_Array *sensor_array);
```

Returns `1` if **more than 3** of the 4 leftmost sensors (indices 0–3) have `.on == 1`, indicating a hard left turn or left branch at a junction.

| Return | Meaning |
|:---:|---|
| `1` | Strong line presence on the left half |
| `0` | No significant left detection |

Must be called **after** `binarizeSensors()`.

```c
binarizeSensors(&sensor_array);
if (detect_left(&sensor_array)) {
    // execute left turn maneuver
}
```

---

### `detect_right`

```c
int detect_right(Sensor_Array *sensor_array);
```

Returns `1` if **more than 3** of sensors at indices 7, 6, 5 have `.on == 1`, indicating a hard right turn or right branch.

| Return | Meaning |
|:---:|---|
| `1` | Strong line presence on the right half |
| `0` | No significant right detection |

Must be called **after** `binarizeSensors()`.

> ⚠️ **Known bug:** The loop checks indices 7, 6, 5 — only **3 sensors** — but the threshold is `sum > 3`, which requires at least 4 active sensors. This condition can **never be true**. The threshold should likely be `> 2`, or the loop extended to include index 4.

```c
binarizeSensors(&sensor_array);
if (detect_right(&sensor_array)) {
    // execute right turn maneuver
}
```

---

## Typical Usage Flow

```
Startup
  │
  ├─ Initialize_Sensor_Array()    // assign weights, set defaults
  ├─ autoCalibrate()              // sweep to record min/max ADC per sensor
  │
  └─ Main Loop  (every ~5 ms)
       │
       ├─ Sync_Sensors()          // DMA buffer → adc_raw
       ├─ processSensors()        // adc_raw → mapped_value (0–1000)
       ├─ binarizeSensors()       // mapped_value → on/off flag
       │
       ├─ count_active_sensors()  // detect junction / lost line
       ├─ detect_left()           // detect left branch
       ├─ detect_right()          // detect right branch
       │
       ├─ get_line_error()        // weighted position error
       ├─ calculate_pid()         // error → motor correction
       └─ set_motor_speed()       // apply correction to motors
```

**Minimal line-follower loop:**

```c
Initialize_Sensor_Array(&sensor_array);
autoCalibrate(&sensor_array, 2000, 40);

while (1) {
    Sync_Sensors(&sensor_array);
    processSensors(&sensor_array);
    binarizeSensors(&sensor_array);

    int active = count_active_sensors(&sensor_array);
    if (active == 0) {
        set_motor_speed(0, 0);
        continue;
    }

    int error      = get_line_error(&sensor_array);
    int correction = calculate_pid(&pid, error, 0.005f);

    set_motor_speed(BASE_SPEED - correction, BASE_SPEED + correction);
    HAL_Delay(5);
}
```
