# voloop — Voltage Output Loop

A lightweight, **hardware-independent** digital power control algorithm library
written in embedded C (C99). The core algorithms are pure numerical
implementations with no dependencies on any MCU peripheral or operating system,
making the library portable across any Cortex-M, DSP, or host platform.

---

## Features

| Component | Description |
|-----------|-------------|
| **PID controller** | Discrete-time position-form PID with clamped integral anti-windup and output saturation |
| **2P2Z compensator** | Type II (2-pole 2-zero) digital compensator — Direct Form I |
| **3P3Z compensator** | Type III (3-pole 3-zero) digital compensator — Direct Form I |
| **Moving-average filter** | Uniform FIR filter, configurable window up to 16 samples |
| **IIR low-pass filter** | First-order exponential smoothing filter |

---

## Repository layout

```
voloop/
├── include/
│   ├── voloop.h               Top-level umbrella header (include this one)
│   ├── voloop_pid.h           PID controller API
│   ├── voloop_compensator.h   2P2Z / 3P3Z compensator API
│   └── voloop_filter.h        Moving-average and IIR filter API
├── src/
│   ├── voloop_pid.c
│   ├── voloop_compensator.c
│   └── voloop_filter.c
├── example/
│   ├── README.md              How to build and run the examples
│   ├── basic_pid_voltage_loop.c
│   └── advanced_compensator_loop.c
└── Makefile
```

---

## Quick start

### 1. Add the library to your project

Copy the `include/` and `src/` directories into your project tree and add the
source files to your build system. No configuration headers, no build-time
defines, and no dynamic allocation are required.

### 2. Include the umbrella header

```c
#include "voloop.h"
```

### 3. Declare and initialise a controller

```c
/* PID example — 5 V Buck converter, 10 kHz control loop */
voloop_pid_t pid;

voloop_pid_init(&pid,
                /* kp */ 0.15f,
                /* ki */ 0.08f,   /* pre-scaled by Ts = 100 µs */
                /* kd */ 0.001f,  /* pre-scaled by 1/Ts */
                /* out_min */ 0.0f,
                /* out_max */ 1.0f);
```

### 4. Call the update function every control period

```c
/* Inside your control-loop ISR (fires at Ts = 100 µs) */
float vout = adc_read_vout();                      /* hardware ADC read */
float duty = voloop_pid_update(&pid, VREF, vout);  /* run PID */
pwm_set_duty(duty);                                /* hardware PWM write */
```

---

## API reference

### PID controller (`voloop_pid.h`)

```c
void  voloop_pid_init(voloop_pid_t *pid,
                      float kp, float ki, float kd,
                      float out_min, float out_max);
void  voloop_pid_reset(voloop_pid_t *pid);
float voloop_pid_update(voloop_pid_t *pid, float setpoint, float feedback);
```

### 2P2Z compensator (`voloop_compensator.h`)

```c
void  voloop_2p2z_init(voloop_2p2z_t *comp,
                       float b0, float b1, float b2,
                       float a1, float a2,
                       float out_min, float out_max);
void  voloop_2p2z_reset(voloop_2p2z_t *comp);
float voloop_2p2z_update(voloop_2p2z_t *comp, float error);
```

### 3P3Z compensator (`voloop_compensator.h`)

```c
void  voloop_3p3z_init(voloop_3p3z_t *comp,
                       float b0, float b1, float b2, float b3,
                       float a1, float a2, float a3,
                       float out_min, float out_max);
void  voloop_3p3z_reset(voloop_3p3z_t *comp);
float voloop_3p3z_update(voloop_3p3z_t *comp, float error);
```

### Filters (`voloop_filter.h`)

```c
/* Moving-average */
void  voloop_ma_init(voloop_ma_t *f, uint8_t size);
void  voloop_ma_reset(voloop_ma_t *f);
float voloop_ma_update(voloop_ma_t *f, float sample);

/* IIR low-pass  y[n] = α·x[n] + (1−α)·y[n−1] */
void  voloop_iir_init(voloop_iir_t *f, float alpha);
void  voloop_iir_reset(voloop_iir_t *f);
float voloop_iir_update(voloop_iir_t *f, float sample);
```

---

## Building and running the examples

```sh
make        # build all example programs into build/
make test   # build and run both examples
make clean  # remove build artifacts
```

See [`example/README.md`](example/README.md) for details on each example and
how to adapt them to real hardware.

---

## Design notes

* **No heap allocation** — every object is declared by the caller and passed by
  pointer; the library never calls `malloc`.
* **No global state** — all state lives inside the caller-provided structs,
  making it safe to run multiple independent control loops simultaneously.
* **C99 with `float`** — suitable for MCUs with a hardware FPU (Cortex-M4F,
  M7, …). For fixed-point targets, replace `float` with `int32_t` scaled
  arithmetic in the source files.
* **Direct Form I compensators** — preferred over Direct Form II for embedded
  use because coefficient sensitivity is lower and saturation can be applied
  directly to the output register.
