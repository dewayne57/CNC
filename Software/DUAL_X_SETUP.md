# Dual-X gantry setup

The firmware exposes one X coordinate to G-code and sends each normal X step
to both DM556 modules. X1 and X2 have separate STEP, DIR and ENA outputs, separate
home switches, separate hard-limit inputs, and their own axle encoders. Both
transmissions must use the same motor steps/revolution, microstep setting, gear
ratio and travel/revolution. Configure direction inversions for mirrored motors.

## Wiring

| Function | X1 | X2 |
| --- | --- | --- |
| STEP | D2 | D26 |
| DIR | D5 | D27 |
| ENA | D8 | D28 |
| Home switch | D23 | D29 |
| Hard-limit input | D30 | D31 |
| Encoder A | D18 | D20 |
| Encoder B | D19 | D21 |

Use the DM556 control wiring described in `README.md`. Switch inputs use
`INPUT_PULLUP`, active LOW, with contacts to signal ground. These defaults are
normally-open and do not detect a disconnected switch cable. Either X hard-limit
input always inhibits motion, including during homing; only home inputs can be
ignored by the homing state machine. Each hard-limit input can represent the
side's limit circuit; route additional end switches through an appropriate circuit.

For the confirmed 5 V open-collector encoders, connect A and B to their assigned
inputs, encoder supply to a suitable regulated 5 V source, and encoder signal
ground to controller signal ground. Internal pull-ups are enabled. Long cables
on a plasma table need a properly designed signal interface, grounding and
noise suppression; weak internal pull-ups alone are not a validated field wiring
solution. External pull-ups must suit the encoder's sink-current rating and
cable capacitance. No encoder signal at the Mega may exceed its 5 V logic supply.

D18-D21 are interrupt-capable on the Mega; they are also shared with Serial1
and I2C, so those peripherals are unavailable with this pin map. See the official
[Mega 2560 pin documentation](https://store.arduino.cc/products/arduino-mega-2560-rev3).
The AVR decoder reads both channels of each pair in a single port read. If pins
are changed, each pair must remain on one input port and both pins must support
external interrupts.

## Calibration that is still required

Edit `Config.h`:

| Setting | Meaning / current value |
| --- | --- |
| `X_MOTOR_FULL_STEPS_PER_REV` | 200 assumes a 1.8-degree motor; verify the motor |
| `CNC_X_MICROSTEPS` | Currently 0 (unconfigured); set the microstep multiplier on both DM556s |
| `X_GEAR_REDUCTION` | 4.0 motor revolutions per axle revolution, as specified |
| `CNC_X_AXLE_TRAVEL_MM` | Currently 0 (unconfigured); set linear travel for one axle revolution |
| `X_ENCODER_SLOTS` | 200 slots on each axle encoder |
| `X_ENCODER_INVERT` | Set independently so positive X gives positive encoder counts |
| `X2_INVERT_DIR` | Set if X2 needs inverted DIR to move the gantry in positive X |

Both unknown calibration values intentionally block X moves, X homing and all
arcs. The startup banner explains which values to set; Y/Z linear motion is
still available. A motor microstep **multiplier** is not the DM556's pulses/rev
number: for a 200-full-step motor, a 3200-pulse/rev setting means 16 microsteps.

The relationships are:

```text
motor_pulses_per_rev = motor_full_steps_per_rev * microsteps
motor_pulses_per_axle_rev = motor_pulses_per_rev * 4
X_steps_per_mm = motor_pulses_per_axle_rev / axle_travel_mm
encoder_counts_per_axle_rev = 200 * 4 = 800
encoder_counts_per_mm = 800 / axle_travel_mm
motor_pulses_per_encoder_count = motor_pulses_per_axle_rev / 800
```

The second factor of four is quadrature decoding, unrelated to the 4:1 gearbox.
The encoders are on the axle **after** the gearbox, so do not multiply their
800 counts by the gear reduction. The decoder counts all A/B changes; this is
the [4X counting convention](https://www.pjrc.com/teensy/td_libs_Encoder.html).
For a rack-and-pinion drive, axle travel is pinion teeth times the rack's linear
tooth pitch. Include any further ratio between the encoder axle and the pinion.

## Monitoring and diagnostics

Each side's measured encoder displacement is compared with that side's actual
emitted command pulses, including pulses emitted independently during homing.
The firmware does not inject corrective steps or run a position servo.

`M114` and `?` append a line such as:

```text
enc X1:100 X2:100 err1:0.00 err2:0.00 invalid1:0 invalid2:0 fault:0
```

Counts are signed displacement since boot, not absolute machine positions.
Errors are **measured minus commanded**, in decoded encoder counts. The second
monitor compares the two following errors; it does not compare raw encoder
positions, because independent homing legitimately moves the sides different
distances. Mechanical home-switch alignment establishes the gantry's square
reference. Axle encoders cannot detect slipping downstream of the encoder.

Defaults are 8 counts maximum following error per side, 8 counts maximum
difference between following errors, and a 25 ms continuous violation before
latching a fault. Monitoring runs approximately every 1 ms from the main loop;
this is not a hard real-time or safety-rated stop. One count equals
`axle_travel_mm / 800`, which sets the finest available feedback resolution.
Tune thresholds for encoder quantization, backlash, dynamic lag and the maximum
acceptable physical error. Homing keeps these checks active.

An impossible two-bit A/B transition increments an invalid-transition counter;
four such transitions on either encoder since boot latch a fault. This detects
some wiring/noise/overrun problems, not every lost encoder edge. Excessive
encoder rates can overwhelm software decoding; test maximum speed on hardware.

| Fault | Meaning |
| --- | --- |
| 0 | No encoder fault |
| 1 | X1 following error |
| 2 | X2 following error |
| 3 | Excessive difference between the two following errors |
| 4 | Too many invalid quadrature transitions |

Any encoder fault cancels linear/arc/homing motion, disables all four drivers,
sets spindle PWM to zero, and latches the executor alarm. Reset is required;
G92, M17 and G28 cannot clear the fault. G92 and homing coordinate assignments
do not reset the encoder-to-command reference and cannot conceal missed motion.
Monitoring also remains active while idle/disabled: moving the gantry manually
can produce an alarm, requiring reset and rehoming.

## Simultaneous X homing

`G28 X0` homes both sides as one logical axis. If either home switch is already
pressed, both sides first move away by the pull-off distance and must release.
Both then seek their switches. Each side stops receiving pulses on its first
contact while the other continues. After both contacts debounce, both retract,
release, and repeat the approach at the slow latch feed. The X coordinate is
assigned only after both slow contacts, followed by a joint pull-off.

The total difference in commanded X1/X2 travel from the start of the X home
cycle is bounded by `X_HOME_MAX_DIFFERENTIAL_MM` (default 5 mm). It is not reset
between the fast seek and slow latch. If one switch never arrives, this limit
or the search-distance/timeout limit stops the cycle. Select an allowance that
your gantry can tolerate, and place the home switches so they define a square
gantry within that allowance. No software home-switch offsets are implemented.
Both hard-limit inputs and the E-stop remain live throughout.

With the existing defaults, homing assigns X=0 at the switches and finishes
at X=3 mm after pull-off. `G28` homes Z, then both X sides, then Y. Configure
the actual table travel and home direction before executing either command.

## Commissioning

1. Set the motor calibration, home travel and directions. Verify both X drives
   turn the gantry in the same logical direction at low speed.
2. With mechanics unloaded, verify each axle revolution produces 800 counts
   and that positive commanded motion increases both counts. Correct encoder
   polarity in configuration if needed, then reset.
3. Observe `err1` and `err2` through forward/reverse motion. Confirm they remain
   within the chosen tolerance; G92 should not change these errors.
4. Verify each home and hard-limit input individually with motion disabled.
   Test paired homing with slightly different home arrival positions and a short
   search distance. Confirm the first side stops while the second completes.
5. Simulate feedback loss and each hard-limit input on a low-speed, unloaded
   test. Confirm both drivers and all other motion stop and the alarm stays latched.
6. Complete electrical/noise and maximum-speed testing before plasma integration.

Torch on/off control, Arc OK, pierce timing, THC and continuous contour blending
are not implemented. The existing spindle PWM output is not a complete plasma
interface; see the plasma integration status in `README.md`.
