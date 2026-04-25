# voloop Examples

Two self-contained examples show how to use the voloop library in a digital
power control loop. Both simulate a synchronous Buck converter plant in
software so they can be compiled and run on any host machine without any
embedded hardware.

## Example 1 — Basic PID voltage loop (`basic_pid_voltage_loop.c`)

Demonstrates the simplest possible voltage regulation loop using a PID
controller:

```
Vref ─►[+]─► PID ─► PWM duty ─► Buck plant ─► Vout
        [-]◄────────────────────────────────────────
```

Relevant API: `voloop_pid_init`, `voloop_pid_update`

Build and run (GCC):

```sh
gcc -Wall -Wextra -I../include \
    ../src/voloop_pid.c \
    basic_pid_voltage_loop.c \
    -o basic_pid_voltage_loop -lm
./basic_pid_voltage_loop
```

## Example 2 — Advanced 3P3Z + IIR filter loop (`advanced_compensator_loop.c`)

Shows a production-style loop with:

- **IIR low-pass filter** on the ADC reading to suppress quantisation noise
- **3P3Z (Type III) compensator** for high-bandwidth, phase-stable control

```
Vref ─►[+]─► 3P3Z ─► PWM duty ─► Buck plant ─► Vout
        [-]◄── IIR filter ◄───────────────────────────
```

Relevant API: `voloop_iir_init`, `voloop_iir_update`,
             `voloop_3p3z_init`, `voloop_3p3z_update`

Build and run (GCC):

```sh
gcc -Wall -Wextra -I../include \
    ../src/voloop_compensator.c \
    ../src/voloop_filter.c \
    advanced_compensator_loop.c \
    -o advanced_compensator_loop -lm
./advanced_compensator_loop
```

## Adapting to real hardware

Replace the two stub functions with your MCU's peripheral calls:

| Stub function          | Replace with                                      |
|------------------------|---------------------------------------------------|
| `plant_adc_read()`     | Read the ADC result register and convert to volts |
| `plant_pwm_write(duty)`| Write `duty × ARR` to the PWM compare register   |

The rest of the loop is hardware-independent and does not need to change.
