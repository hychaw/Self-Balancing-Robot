# Hardware and connections

The final robot uses a two-wheel differential-drive chassis with the controller, drivers, motors, encoders, and battery onboard. The component descriptions follow the team's report; the pin map below is checked against the [main firmware](../firmware/main/main.ino).

| Part | Function |
| --- | --- |
| Arduino Nano 33 BLE Sense Rev2 | Runs the controller and BLE peripheral |
| Onboard BMI270 IMU | Supplies acceleration and angular-rate readings |
| Two DRV8871 driver boards | Drive each 12 V DC motor from Nano control signals |
| Two 12 V Pololu DC gearmotors with quadrature encoders | Actuation and wheel-motion feedback |
| Two-wheel chassis | Mechanical inverted-pendulum platform |
| Onboard battery pack | Mobile power source |

## Main-sketch pin map

| Signal | Nano pin |
| --- | --- |
| Debug timing output | D4 |
| Left driver `IN1`, `IN2` | D5, D6 |
| Left encoder A, B | D7, D8 |
| Right driver `IN1`, `IN2` | D9, D10 |
| Right encoder A, B | D11, D12 |

The report describes an onboard 12 V battery pack, a common electrical ground, and voltage dividers on the encoder signal lines because the Nano uses 3.3 V logic. Check the actual wiring and signal levels before connecting or powering a rebuilt robot. These notes document the project hardware; they are not a complete wiring schematic.
