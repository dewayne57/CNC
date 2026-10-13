# Mega 2560 G-code executor

An Arduino sketch for three logical axes driven by four external DM556 STEP/DIR
modules: X1, X2, Y and Z. The X gantry uses 4:1 reductions, two 200-slot axle
encoders, and independently homed sides.
Open `Software.ino` in Arduino IDE, select **Arduino Mega or Mega 2560**, and
upload. No third-party libraries are required. Serial uses **115200 baud**.
**X calibration is intentionally incomplete:** set the microsteps and axle
travel in `Config.h` before X motion or arcs can run. See `DUAL_X_SETUP.md` for
the full pin map, calibration, encoder diagnostics and commissioning procedure.

The default pin allocation matches the nets in `Hardware/CNC.kicad_sch`.
Every pin in `Config.h` has a `CNC_*_PIN` compiler define, so alternate wiring
can be selected without editing the firmware. For example,
`-DCNC_Y_STEP_PIN=32` changes only the Y step output. Polarity and motion
calibration remain configurable in `Config.h`.

## Files

| File | Responsibility |
| --- | --- |
| `Software.ino` | Setup and cooperative main loop |
| `Config.h` | Pins, polarity, calibration, feed and pulse settings |
| `SerialProtocol.*` | Bounded line input, replies, real-time controls |
| `GCodeParser.*` | Decimal word parsing and syntax validation |
| `Executor.*` | Command dispatch, modal state, dwell and alarm handling |
| `Motion.*` | Coordinated XYZ integer DDA stepping and position tracking |
| `MotionProfile.*` | Trapezoidal/triangular acceleration and deceleration |
| `ArcMotion.*` | Plane selection, arc geometry, helical interpolation and segmentation |
| `Homing.*` | Switch seek, debounce, slow latch and pull-off state machine |
| `GantryHoming.*` | Paired X homing with independent switch stops and squaring bound |
| `XGantry.*` | Separate X1/X2 outputs and physical command pulse counters |
| `EncoderFeedback.*` | Interrupt quadrature decoding, following-error and skew faults |
| `Spindle.*` | Optional PWM and direction outputs |
| `Display.*` | 20x4 parallel LCD status and position display |
| `examples/square.gcode` | Slow 10 mm square without spindle activation |
| `examples/arcs.gcode` | Quarter arcs, full circle and helix without spindle activation |
| `tests/` | Host simulation tests with fake pins, switches and time |

The LCD shows the current operation (`IDLE`, `MOVING`, `HOMING`, `DWELL`,
`SPINDLE`, or `ALARM`) and live X/Y/Z coordinates. It refreshes one row every
100 ms so display traffic remains bounded while motion is running.

## DM556 wiring and calibration

Default assignments below use common-cathode control wiring: each driver's
`PUL-`, `DIR-`, and `ENA-` connect to control ground. The listed outputs connect
to the corresponding `+` terminals through a suitable 5 V signal buffer if
required by the module's input current. These are logic connections, not motor
power connections. Do not connect a motor supply or 24 V control signal to the Mega.

| Signal | X1 | X2 | Y | Z |
| --- | --- | --- | --- | --- |
| PUL+ / STEP | D30 | D30 | D33 | D36 |
| DIR+ | D31 | D31 | D34 | D37 |
| ENA+ | D32 | D32 | D35 | D38 |
| Home input | D2 | D3 | D4 | D5 |
| Separate hard-limit input | D6 | D7 | D8 | D9 |
| Encoder A / B | D22 / D23 | D24 / D25 | None | None |
| Spindle PWM / direction | D50 / D51 | — | — | — |
| LCD control | RS D39, RW D40, E D41 | — | — | — |
| LCD data | D42-D49 | — | — | — |

X1/X2 STEP, DIR and ENA nets are shared in the schematic. Each Y/Z enable
output is separate to avoid loading one Mega pin with multiple input
optocouplers. Verify your exact module's input voltage, current, pulse mode,
active edge and enable polarity. Select **5 V control**, **pulse/direction**,
and **rising-edge pulses**, where configurable. Defaults assume LOW enables
the driver; change `ENABLE_ACTIVE_LEVEL` if the module or buffer differs.
Leaving ENA disconnected means software cannot release the motor holding torque.
Provide appropriate hardware biasing if drivers must stay disabled during reset.

The sketch provides 5 us HIGH pulses, at least 250 us between step events, 5 us
direction setup, and 10 us enable setup before changing direction. Leadshine's
[DM series installation manual](https://www.leadshine.com/upfiles/downloads/6a692cdc3bb89c64adcf1023824d34f0_1660188836183.pdf)
specifies 2.5 us minimum HIGH/LOW widths for DM556 and 5 us setup intervals.
DM556-labelled variants can differ; check the manual supplied with your modules.

Set microstep DIP switches and motor current on each driver. Then calculate:

```text
steps_per_mm = driver_pulses_per_motor_revolution * motor_revolutions_per_screw_revolution
               / screw_travel_mm_per_revolution
```

X steps/mm is derived from motor full steps, microsteps, 4:1 reduction and axle
travel. Y=80 and Z=400 steps/mm remain placeholders, not machine calibration.
Set `INVERT_DIR` per axis and `X2_INVERT_DIR` after testing direction. Set feed rates conservatively
for your mechanics. Motion ramps up and down using `ACCELERATION_MM_S2` (default
50 mm/s squared). Short moves use a triangular speed profile. Every move and arc
chord starts and ends at zero speed; there is no look-ahead or junction blending.
Arc segments can therefore be noticeably slower than the requested feed.
The 4000-event/s cap can also reduce commanded feed.
Loop overhead and serial traffic can also slow motion; this is not a timer-driven
production controller.

## Interlocks and spindle

D22 is an active-LOW stop input. Switch assignments are listed above. All use
`INPUT_PULLUP`: a contact to GND asserts the input. E-stop always stops all axes,
disables drivers, stops spindle PWM, and latches an alarm. A limit does the same
during normal motion or dwell. While idle, a limit disables drivers/spindle and
blocks normal commands but allows homing from an active home switch. X hard
limits D30/D31 block all homing and cannot be masked. Homing handles selected switches as
described below; unselected switches still stop the cycle. This normally-open
scheme does not detect broken switch wiring. Clear the cause and reset the board
after a latched alarm; a subsequent G28 can release an initially active home switch.

D9 is optional spindle PWM; D10 is spindle direction (HIGH for M3). Use an
appropriate spindle interface, never a motor directly. `S` maps RPM linearly
to PWM against `MAX_SPINDLE_RPM`; there is no speed feedback or reversal delay.

`!` and M112 are software stops, not safety-rated emergency stops. A machine
requires an independent hardware emergency stop. Disabling a vertical axis can
release its load. Reported XYZ position counts commanded pulses; X encoder
feedback monitors actual axle displacement separately and latches a stop on
mismatch. Y and Z still have no encoder feedback. Re-establish the origin after
reset, a stop or manual movement.

## Commands

| Command | Behavior |
| --- | --- |
| G0 X Y Z | Coordinated rapid positioning at configured rapid feed |
| G1 X Y Z F | Coordinated linear move, F in current units/minute |
| G2 / G3 X Y Z I J K F | Clockwise / counterclockwise center-format arc |
| G2 / G3 X Y Z R F | Radius-format arc; positive R is minor, negative R is major |
| G4 P | Dwell in milliseconds, 0 through 3600000 |
| G17 / G18 / G19 | Select XY / ZX / YZ arc plane |
| G20 / G21 | Inches / millimeters for subsequent coordinates and F words |
| G28 | Home all axes in configured order |
| G28 X0 Y0 Z0 | Home only the listed axes; omit unwanted selectors |
| G90 / G91 | Absolute / incremental coordinates |
| G92 X Y Z | Set selected reported coordinates without moving |
| M3 S / M4 S | Run spindle clockwise / counterclockwise, S in RPM |
| M5 | Stop spindle |
| M17 | Enable all drivers |
| M18 / M84 | Disable all drivers |
| M114 | Report logical XYZ millimeters plus X encoder counts, errors and fault code |
| M400 | Synchronization acknowledgement; earlier moves already finish before ok |
| M112 | Latch emergency-stop alarm |
| ! | Immediate stop request without waiting for a line terminator |
| ? | Position/alarm report without a line terminator or ok |

At boot: millimeters, absolute mode, G1 modal motion, F300 mm/min, logical
position zero, XY arc plane, drivers disabled, spindle off. Motion lines reuse
G0/G1/G2/G3; modal arcs still require center offsets or R on each line.
F-only lines update modal feed. Changing units preserves the existing physical
feed until a new F word arrives. Omitted axes retain their position.

Send **one line and wait for `ok` before the next**. A linear move, complete arc,
homing cycle or dwell receives `ok` only when complete. `error: ...` rejects a
line or reports a cycle failure; stop streaming and resolve it.
There is no command queue or resend protocol. M114 and real-time controls work
during movement, but unsolicited queries may affect pulse timing. A stop aborts
the active command without its normal ok. Reset is required to clear an alarm.

Use LF, CR or CRLF. Lines are limited to 127 characters excluding terminators.
An empty line is ignored; comments-only lines receive ok. Both `(comments)` and
`; comments` are supported. Commands are case-insensitive and spaces optional.
`!` and `?` remain active even within comments. A `!` discards the partial line
through its next terminator. Overflow or invalid bytes discard the whole line.

This is a deliberate G-code subset: one G or M word per line. Work offsets
G54-G59, absolute arc-center modes, compensation, canned cycles, multi-turn arcs,
line numbers, checksums, exponent notation and multiple-command blocks are
rejected. Configure the CAM postprocessor for this subset;
do not send arbitrary CAM output unchanged.

## Arcs

G2 is clockwise and G3 is counterclockwise, viewed from the positive normal of
the selected plane. G17 uses I/J offsets, G18 uses K/I, and G19 uses J/K. An
offset on the remaining axis is rejected. I/J/K are always offsets from the
start of the arc, regardless of G90/G91; omitted in-plane offsets are zero.
Endpoint coordinates follow G90/G91 and all arc lengths follow G20/G21.
Moving the perpendicular axis produces a helix.

Use either center offsets or R, never both. Positive R selects a sweep up to
180 degrees, negative R selects the larger sweep. Full circles require center
offsets and unchanged in-plane endpoints; R circles are ambiguous and rejected.
Center-format start/end radii must agree within `ARC_RADIUS_TOLERANCE_MM`.
These arc conventions follow the [LinuxCNC G-code reference](https://www.linuxcnc.org/docs/2.8/html/gcode/g-code.html).

Arcs are approximated with chords, using both `ARC_CHORD_ERROR_MM` (0.02 mm)
and `ARC_MAX_SEGMENT_MM` (1 mm). Step rounding and accepted radius mismatch
add to geometric error. The last chord uses the exact requested endpoint,
rounded to steps. An arc requiring more than `ARC_MAX_SEGMENTS` (10000) is
rejected before motion. The entire circle's bounding box must fit within the
numeric coordinate bound. Invalid geometry does not change modal feed/motion.
Each chord accelerates and decelerates independently; this implementation favors
exact stopping over smooth continuous contouring.

## Homing

Here **G28 means switch-based homing**, not a move to a stored reference point
or an intermediate waypoint. `G28` homes Z, then the X pair, then Y. `G28 X0`
homes both X sides together; each stops independently at its own home switch.
`G28 X0 Y0` homes the X pair and Y. Selectors must be numeric zero; bare axis
letters and nonzero coordinates are rejected. G90/G91, G20/G21 and modal feed
do not alter homing, whose configuration is always in millimeters.

Configure `HOME_ORDER`, `HOME_DIRECTION`, `HOME_MAX_TRAVEL_MM`,
`HOME_SEEK_MM_MIN`, `HOME_LATCH_MM_MIN`, `HOME_PULL_OFF_MM`, and
`HOME_SWITCH_POSITION_MM` before running it. Defaults seek X/Y negative and
Z positive, with 300/300/100 mm maximum search distances and a 3 mm pull-off.
These are example values, not measured machine travel. Normal moves still have
no physical soft travel limits, and homing is not required before normal moves.

The spindle is stopped at entry. Each selected axis (or X pair) runs this sequence:

1. If its switch is already pressed, move away by the pull-off distance and
   require a stable release.
2. Seek the switch at the fast homing feed, bounded by maximum search distance.
3. Halt pulses on first contact (independently for each X side), require debounced
   active switches, then retract together.
4. Require stable release, then approach again at slow latch feed. The second
   search is bounded by twice the pull-off distance.
5. On debounced contact, assign the switch coordinate, then pull away and
   require stable release before advancing to the next axis.

Homing legs use acceleration; switch contact stops pulses immediately without a
deceleration ramp. The switch must remain in its expected state for
`HOME_DEBOUNCE_MS`. Missing contact, a stuck switch or a phase exceeding
`HOME_TIMEOUT_MS` fails the cycle and latches an alarm. Selected axes awaiting
their turn may have active switches; completed/unselected axes are monitored
normally. E-stop and both X hard limits remain active throughout. X homing fails
if cumulative commanded side-to-side correction exceeds
`X_HOME_MAX_DIFFERENTIAL_MM`. Encoder following-error checks remain active during
homing, including when only one side is still moving. Both home switches must
be aligned mechanically to establish a square gantry.

Reported coordinates after successful homing include final pull-off:
`switch_position - home_direction * pull_off`. Defaults therefore finish at
X=3, Y=3, Z=-3 mm, with the physical switch locations assigned zero. Homing
overwrites G92 coordinates on selected axes. Verify the switch polarity and
seek direction with short configured search distances before a full cycle.

## Build and bench checks

```powershell
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:mega Software
```

Run those commands from the repository root. Before connecting mechanics, verify
STEP, DIR and ENA with a logic analyzer, including an enable transition followed
by a direction reversal. Check switch and stop handling during a long move.
Then calibrate at low feed with the spindle off and establish an origin using G92.

Useful serial checks (wait for each reply):

```gcode
G21
G90
G92 X0 Y0 Z0
G1 X1 Y0.5 F60
M114
G91
G1 X-1 Y-0.5
M114
```

The reports should be approximately `(1, 0.5, 0)` and `(0, 0, 0)`, subject to
step rounding. `G1 X1 X2`, `G1 F0`, `G2 X1`, and `G90 G21` must return errors
without movement. During `G1 X100 F60`, send `!` and verify motion and spindle
stop and subsequent motion is rejected until reset.

Host simulation tests can be run from the repository root with a Windows GCC
compiler (its bin directory must contain the compiler's companion tools):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Software/tests/run.ps1 -Compiler C:/path/to/g++.exe
```

Tests cover parser validation, acceleration symmetry and short moves, pulse
counts, timer rollover, CW/CCW arcs, signed radii, full circles, all three planes,
helices, invalid geometry, homing order, initially pressed/missing/stuck switches,
interlocks, unit/mode handling, and cancellation. Host simulation and compilation
do not verify real switch behavior, motor torque, timing under load or wiring;
the hardware bench checks above are still needed.

The dual-X tests additionally cover 4:1 pulse/count scaling, independent home
arrival, squaring bounds, a missing switch, quadrature reversal, invalid transitions,
transient versus sustained error, X2 feedback loss, hard limits during homing,
and preservation of the encoder reference across G92. The native test runner
uses explicit fixture calibration, not the unconfigured machine defaults.

## Plasma integration status

This implements gantry motion and encoder monitoring; it does not yet implement
a plasma torch interface, Arc OK handshake, pierce sequencing, torch-height
control, or continuous contour blending. D9/D10 still have their original spindle
PWM/direction semantics. Do not treat those as a completed torch controller.
Every encoder alarm forces D9 PWM off, but physical torch shutdown requires the
appropriate isolated machine interface and independent stop circuit. The current
exact-stop arc segments and feed limits also need revisiting for plasma cutting.
