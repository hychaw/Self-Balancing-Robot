# Control system

The robot is an inverted pendulum on two driven wheels. It moves the wheelbase under the tilting body to keep the center of mass supported. The [integrated final sketch](../firmware/main/main.ino) implements an outer position loop and an inner balance loop; the team's report explains why position control replaced an earlier velocity-loop approach for the final demo.

## Tilt estimate and balance loop

At startup, the sketch averages 500 gyroscope samples to estimate x-axis bias and initializes the filter from an accelerometer angle. During operation it computes the accelerometer angle from `atan2(y, z)` and applies:

```text
filtered_angle = 0.98 × (previous_filtered_angle + bias_corrected_gyro_x × dt)
               + 0.02 × accelerometer_angle
```

The inner angle controller uses filtered-angle error, x-axis angular rate as its derivative term, and a clamped integral term. Its output is a signed **base PWM**. Driving the wheels in response to body tilt keeps the platform under the body. The complementary filter reduces accelerometer noise while limiting integrated gyro drift.

## Encoder position loop

Encoder interrupts count both quadrature channels. The slower outer loop uses the average of left and right counts as `currentPosition`. A slew limit moves `targetPositionSmoothed` toward the command target. Position error and a derivative adjusted for the moving target feed the outer PID, whose output is a **target lean angle**, limited by `MAX_ANGLE`. The angle loop then turns that requested lean into motor effort. This separation lets the position controller request motion while the inner loop continues balancing.

| Main-sketch value | Source value | Context |
| --- | ---: | --- |
| Complementary-filter weight `k` | `0.98` | Gyro-integrated estimate weight |
| Angle `KP`, `KD`, `KI` | `8.25`, `0.5`, `45` | Inner-loop terms |
| Position `PC_KP`, `PC_KD`, `PC_KI` initializers | `0.0015`, `0.00075`, `0.0003` | Outer-loop values at initialization |
| Position `PC_KD` in `rampIdle` | `0.0015` | The idle-state branch assigns this while the loop runs |
| Position-loop interval | `40 ms` | Loop executes when elapsed time exceeds the threshold |
| Normal `MAX_ANGLE` | `3°` | Flat-ground limit; ramp states can increase it |
| Normal target-position slew rate | `900` encoder counts/s | Ramp states can change it |
| Left/right deadband values | `42` / `42` PWM counts | Fixed values; automatic routine is disabled in `setup()` |

These are source-code settings, not measured performance figures. The `PC_KD` reassignment matters when reproducing the active flat-ground tuning.

## Actuation and heading

The signed base PWM is mixed with `headingCorrection`: it is added to the left command and subtracted from the right. `applyDeadband()` adds or subtracts the configured threshold for nonzero PWM to overcome the motors' low-command dead zone. `imbalanceMultiplier` is `1.00` in the idle turn state and `1.07` during a staged turn. This is an empirical correction for unequal wheel response, not wheel-speed feedback control.

For heading, the sketch integrates the z-axis gyro rate and compares it with `targetHeading`. It calculates a z-axis startup average but **does not use that average** in the operating heading calculation. Instead, it subtracts the measured fixed offset `0.149990639` from `gyro_z`; the report says the startup z-axis estimate caused inconsistent correction. Left and right commands initiate two 45° target-heading steps with a timed wait between them, while differential PWM makes the correction.

## Ramp mode

An `A`-prefixed BLE command selects ramp mode. The integrated sketch has climb, wait, descend, and complete states, including staged climb states named for 10° and 15°. Its state machine changes position targets, slew rate, wait times, and the allowed lean angle; it does not measure ramp inclination or select a variant automatically. The [5°](../firmware/ramp_variants/ramp_5deg/ramp_5deg.ino), [10°](../firmware/ramp_variants/ramp_10deg/ramp_10deg.ino), and [15°](../firmware/ramp_variants/ramp_15deg/ramp_15deg.ino) sketches contain separate trial tuning. Their names identify the tests, not verified traversal outcomes.
