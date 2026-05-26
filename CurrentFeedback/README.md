# Brushed DC Current Feedback Experiment

This project is a small Arduino Mega / PlatformIO experiment for a brushed DC motor.

It generates a PWM signal, samples shunt current in sync with the PWM pulse, and lets you compare 2 modes:

- `OPEN_LOOP`: direct duty setpoint
- `CLOSED_LOOP`: duty setpoint plus a simple current-based correction

It also has a VFD display to show the current mode and control parameters.

This is not a true speed loop. The closed-loop mode uses motor current as a load indicator and adjusts duty upward when current rises.



## Hardware

- Board: Arduino Mega 2560
- PWM output: `D11` (`OC1A`)
- Current sense input: `A0`. The current sense in the sketch assumes:
  - Shunt resistor: `1 Ohm`
  - ADC reference: internal `1.1 V`
  - Measurable current range: about `0` to `1.1 A`

- Scope trigger: `D2`
- VFD display: `Serial1` at `19200` baud
- VFD wiring: Mega `TX1` (`D18`) to display RX, and common GND



## Build

This project uses PlatformIO.

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```



## Serial Commands

Use the USB serial monitor at `115200` baud.

- `o`: set mode to`OPEN LOOP`

- `c`: set mode to`CLOSED LOOP`

- `w`: increase duty by `1%`

- `W`: increase duty by `10%`

- `s`: decrease duty by `1%`

- `S`: decrease duty by `10%`

- `1`: store calibration point 1

- `2`: store calibration point 2

  

## Calibration Flow

Typical use:

1. Start in `OPEN LOOP`.
2. Set a base duty and let the motor settle. Measure motor RPM
3. Press `1` to store point 1.
4. Increase load and duty manually to achieve the same RPM at higher load.
5. Press `2` to store point 2.
6. Switch to `CLOSED LOOP`.

The sketch calculates a simple correction gain in duty-per-amp from those two points.



## VFD Display

The VFD on `Serial1` shows:

- Mode
- Measured current
- Duty setpoint
- Commanded output duty for `CLOSED LOOP` mode



## Notes

- PWM frequency is `10 kHz`.
- ADC sampling is synchronized near the middle of the PWM on-time.
- The code writes directly to AVR timer and ADC registers for timing control.
