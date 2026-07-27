# Heading node CAN protocol

A Teensy-based node with a u-blox NEO-M9N (GPS) and an MPU-9250 (9-axis IMU)
that runs edge fusion and broadcasts a finished heading the HMI can consume
directly.

Source address `0xE1`, chosen clear of every source seen on the boat's bus.
Three proprietary IDs, all verified collision-free against the July 2026
captures.

All multi-byte fields are **little-endian** to match the existing decoder,
which uses `struct.unpack('<...')` throughout. Angles are offset-encoded so an
unsigned 16-bit field can carry a signed value without sign-extension bugs --
the same convention the boat's J1939 pitch field uses (and which bit us once
already).

Byte order and scaling are the contract. The sketch and the decoder must agree
exactly; both are generated from this file, so change them together.

---

## 0x18FF80E1 -- Fused heading (10 Hz)

The message the autopilot actually steers by. This is the whole point of the
node: a trustworthy heading at any speed, not just when COG locks above
1.6 mph.

| Bytes | Field | Type | Encoding | Range |
|-------|-------|------|----------|-------|
| 0-1 | heading | u16 LE | deg x 100 | 0 .. 359.99 |
| 2-3 | heading_sigma | u16 LE | deg x 100 | 0 .. 655.35 |
| 4-5 | yaw_rate | u16 LE | (deg/s x 100) + 32768 | -327.68 .. +327.67 |
| 6 | source | u8 | 0=NONE 1=MAG 2=GPS 3=FUSED | |
| 7 | status | u8 | bit0 valid, bit1 mag_cal_ok, bit2 gps_ok, bit3 holding | |

`heading` is the fused estimate, always 0..360. `heading_sigma` is the filter's
1-sigma confidence in degrees -- the HMI already has a `heading_sigma` signal
and gates engagement on it, so this feeds straight in. `yaw_rate` is the
gyro-derived turn rate, offset by 32768 so straight-line = 32768.

`source` tells the HMI where the heading came from this cycle: pure
magnetometer at rest, GPS course at speed, or the blended estimate. `status`
bit0 is the master validity flag -- if clear, the HMI must not steer by this
message.

## 0x18FF81E1 -- Attitude (10 Hz)

Pitch and roll for the sea-state vs. bad-sensor attribution the heading monitor
already does. Roll was never available before (nothing on the boat published
it), so this finally feeds `add_roll()`.

| Bytes | Field | Type | Encoding | Range |
|-------|-------|------|----------|-------|
| 0-1 | pitch | u16 LE | (deg x 100) + 18000 | -180.00 .. +180.00 |
| 2-3 | roll | u16 LE | (deg x 100) + 18000 | -180.00 .. +180.00 |
| 4-5 | accel_mag | u16 LE | milli-g | 0 .. 65.535 g |
| 6 | reserved | u8 | | |
| 7 | status | u8 | bit0 valid | |

Pitch and roll use a +18000 offset (deg x 100) so level = 18000. `accel_mag` is
the total acceleration magnitude in milli-g; well above 1000 sustained means
the boat is being thrown around, which corroborates high heading sigma as sea
state rather than a sensor fault.

## 0x18FF82E1 -- Node health (1 Hz)

Slow housekeeping so the HMI can show why the node is or isn't trustworthy.

| Bytes | Field | Type | Encoding | Range |
|-------|-------|------|----------|-------|
| 0 | fix_type | u8 | 0=none 2=2D 3=3D 4=GNSS+DR | |
| 1 | num_sats | u8 | count | 0 .. 255 |
| 2-3 | hdop | u16 LE | x 100 | 0 .. 655.35 |
| 4 | mag_cal | u8 | 0=uncal 1=poor 2=ok 3=good | |
| 5 | temp | s8 | deg C | -128 .. 127 |
| 6-7 | uptime | u16 LE | seconds (wraps) | |

`mag_cal` is the magnetometer calibration quality. If this sits at 0 or 1, the
magnetometer heading is no better than the stuck compass we already have -- the
HMI should warn, and the node should weight GPS course more heavily.

---

## Why offset encoding for the angles

A signed value packed as raw two's-complement into a `<H` unpack reads as a
huge positive number; packing as `<h` works but then every consumer must know
which fields are signed. Offset encoding sidesteps both: everything unpacks as
`<H`, and the consumer subtracts a constant. It is the same scheme J1939 uses
for pitch (offset 200 deg), and matching it keeps the decoder uniform.

Center points:
- heading: none (always 0..360, no offset needed)
- yaw_rate: 32768 (0.01 deg/s per bit)
- pitch/roll: 18000 (0.01 deg per bit)

## Update rates and bus load

At 10 Hz for two 8-byte messages plus 1 Hz for a third, the node adds about 21
frames/s -- negligible on a bus already carrying thousands. The 10 Hz rate
matches the HMI's health tick and gives the autopilot fresh heading every
100 ms, comfortably faster than the rudder can move.

## Failure behavior

If the node loses GPS and the magnetometer is uncalibrated, it sets
`source=NONE` and clears `status` bit0. The HMI treats a stale or invalid
heading node exactly as it treats a lost COG lock today: hold if engaged and
moving, center if stopped. So a dead node degrades to the current
COG-primary behavior rather than breaking anything.
