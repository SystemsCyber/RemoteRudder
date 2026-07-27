/*
 * heading_node.ino
 *
 * Teensy 4.0 heading node for the RemoteRudder boat.
 *   - u-blox NEO-M9N (GPS course + speed)   over Serial (UBX)
 *   - MPU-9250 (gyro + accel + magnetometer) over I2C / SPI
 *   - broadcasts a fused heading on CAN for the HMI to steer by
 *
 * Edge fusion: the filter runs here, next to the IMU where the timing is
 * clean, and the node emits a finished heading. The HMI consumes it directly
 * via the three IDs defined in PROTOCOL.md. Keep this sketch and the HMI
 * decoder in sync -- both implement the same byte layout.
 *
 * Why this node exists: the boat's built-in magnetic compass is stuck (never
 * reads above ~264 deg in the captured runs), which forced the HMI to COG-only
 * steering that dies below ~1.6 mph. A calibrated magnetometer plus GPS course
 * gives a usable heading at any speed -- but ONLY if the magnetometer is
 * actually calibrated (see the calibration section). An uncalibrated MPU-9250
 * near a boat engine is no better than the compass we are working around.
 *
 * Board: this is the "Secure Gateway or CAN Conditioner" Rev 2 (2020),
 * populated with its GPS and Accelerometer daughter boards -- NOT a separate
 * node. Per that schematic:
 *   - CAN1 (vehicle transceiver U5): CAN1TX = D22, CAN1RX = D23
 *   - MPU-9250 on Wire1 (I2C1): SDA1 = D17, SCL1 = D16, own 4.7k pullups,
 *     plus an INT line to the Teensy
 *   - NEO-M9N on the primary I2C (Wire, SDA0=D18/SCL0=D19, shared with the
 *     ATECC608A), and also broken out to a UART (GPS_TX/GPS_RX) + PPS
 *
 * IMPORTANT hard-iron warning: because the IMU shares this board with two CAN
 * transceivers, the OKI-78SR switching converter, and four LEDs, the
 * magnetometer sits inches from several time-varying magnetic sources. The
 * hard-iron field is therefore partly a function of what the board is doing
 * (LEDs on, converter load), not just the boat. Calibrate with the board in
 * its normal operating state -- powered, LEDs in their usual condition, CAN
 * active -- and expect the calibration to be less clean than a standalone
 * magnetometer would give. If heading quality stays poor after calibration,
 * the fix is physical: move the M9N/MPU onto a short mast or standoff away
 * from the board, which is the standard remedy for this exact problem.
 *
 * Libraries (install via Arduino Library Manager unless noted):
 *   - FlexCAN_T4          (Teensy built-in / https://github.com/tonton81/FlexCAN_T4)
 *   - SparkFun u-blox GNSS Arduino Library (M9N over I2C or UART)
 *   - MPU9250 (bolderflight) or equivalent on Wire1; the IMU read is isolated
 *     in readIMU() so a different driver is a small change.
 *
 * This sketch is deliberately dependency-light in its fusion math: a
 * complementary filter, not a full EKF. It is easy to reason about, runs in
 * microseconds, and is good enough for a boat heading. If you later want an
 * EKF it drops into the same readIMU()/readGPS()/fuse() structure.
 */

#include <FlexCAN_T4.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// CAN IDs (extended/29-bit). Must match PROTOCOL.md and the HMI decoder.
static const uint32_t CAN_ID_HEADING  = 0x18FF80E1;
static const uint32_t CAN_ID_ATTITUDE = 0x18FF81E1;
static const uint32_t CAN_ID_HEALTH   = 0x18FF82E1;

static const uint32_t CAN_BITRATE = 250000;  // match the boat bus

// Pin assignments from the Rev 2 gateway schematic.
// MPU-9250 is on Wire1 (I2C1); the NEO-M9N is on the primary Wire, but the
// SparkFun library is happiest on the M9N's UART, which is also broken out.
static const int PIN_MPU_INT   = 9;    // MPU-9250 INT (D9, adjust if different)
static const int PIN_GPS_INT   = 10;   // M9N INT
static const int PIN_GPS_RESET = 11;   // M9N !RESET
static const int PIN_GPS_PPS   = 5;    // M9N PPS (for future time-sync; unused here)
#define GPS_SERIAL Serial5             // GPS_TX/GPS_RX = RX5/TX5 (D20/D21)
#define IMU_WIRE   Wire1               // SDA1=D17, SCL1=D16

// Loop timing
static const uint32_t IMU_PERIOD_US   = 5000;   // 200 Hz IMU sampling
static const uint32_t FUSE_PERIOD_US  = 10000;  // 100 Hz filter update
static const uint32_t TX_FAST_US      = 100000; // 10 Hz heading + attitude
static const uint32_t TX_HEALTH_US    = 1000000;// 1 Hz health

// Complementary filter time constant. Higher = trust the gyro longer (smoother
// but drifts); lower = trust the absolute reference (mag/GPS) sooner (noisier
// but no drift). 5 s is a reasonable boat default.
static const float TAU = 5.0f;

// Speed above which GPS course is trusted as the absolute heading reference
// (m/s). Below this, fall back to the magnetometer. 0.7 m/s ~ 1.6 mph, the
// same threshold the HMI uses for COG.
static const float GPS_COURSE_MIN_MPS = 0.7f;

// Magnetic declination for your area, degrees East positive. Horsetooth /
// Fort Collins is about +8 deg E. Set from https://www.ngdc.noaa.gov/geomag/
static const float MAG_DECLINATION_DEG = 8.0f;

// ---------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// ---------------------------------------------------------------------------
// Magnetometer calibration
//
// THIS IS THE CRUX. An MPU-9250 reads garbage until hard-iron (offset) and
// soft-iron (scale) corrections are applied -- and on THIS board the problem is
// worse than usual, because the magnetometer shares the PCB with two CAN
// transceivers, the OKI-78SR switching converter, and four LEDs, all magnetic
// sources inches away and some of them switching.
//
// Calibrate with the board in its NORMAL operating state: powered from the
// boat, CAN active, LEDs in their usual condition. Calibrating on the bench in
// a different power/LED/CAN state bakes in the wrong hard-iron offset.
//
// Until you paste real values below, magCalQuality stays low and the node
// leans on GPS course. Do the calibration on the boat, engine running, away
// from the dock, rotating slowly through all orientations (figure-eights).
//
// If heading quality stays poor after a proper calibration, the fix is
// physical: relocate the M9N/MPU daughter boards onto a standoff or short mast,
// away from the converter and transceivers. No software correction beats moving
// the sensor out of a switching magnetic field.
// ---------------------------------------------------------------------------

struct MagCal {
  float offset[3] = {0, 0, 0};   // hard-iron, uT
  float scale[3]  = {1, 1, 1};   // soft-iron (diagonal), unitless
  bool  valid = false;           // set true once you paste real values
};

// Paste the output of calibrateMag() here, then set valid = true.
MagCal magCal = {
  {0.0f, 0.0f, 0.0f},
  {1.0f, 1.0f, 1.0f},
  false
};

// ---------------------------------------------------------------------------
// Fusion state
// ---------------------------------------------------------------------------

struct State {
  float heading = 0;        // deg, 0..360, fused
  float headingSigma = 99;  // deg, filter confidence (starts pessimistic)
  float yawRate = 0;        // deg/s
  float pitch = 0, roll = 0;// deg
  float accelMag = 1.0f;    // g

  uint8_t source = 0;       // 0 NONE, 1 MAG, 2 GPS, 3 FUSED
  bool valid = false;
  bool holding = false;     // GPS lost at speed, coasting on gyro

  // GPS
  uint8_t fixType = 0;
  uint8_t numSats = 0;
  float hdop = 99;
  float gpsCourseDeg = 0;
  float gpsSpeedMps = 0;
  bool gpsCourseValid = false;

  // health
  uint8_t magCalQuality = 0; // 0 uncal .. 3 good
  int8_t tempC = 0;
  uint16_t uptime = 0;
} S;

uint32_t bootMillis = 0;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static inline float wrap360(float d) {
  while (d < 0)    d += 360.0f;
  while (d >= 360) d -= 360.0f;
  return d;
}

// Shortest signed difference a-b on the circle, in (-180, 180].
static inline float angDiff(float a, float b) {
  float d = a - b;
  while (d > 180)  d -= 360;
  while (d <= -180) d += 360;
  return d;
}

static inline uint16_t u16le_scaled(float value, float scale, float offset,
                                    float lo, float hi) {
  float v = value * scale + offset;
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (uint16_t)(v + 0.5f);
}

// ---------------------------------------------------------------------------
// Sensor reads -- isolated so a different driver is a localized change.
//
// These are stubs with the shape the rest of the sketch expects. Wire them to
// your actual MPU9250 / u-blox libraries. Each returns true on a fresh sample.
// ---------------------------------------------------------------------------

bool readIMU(float gyro[3], float accel[3], float mag[3]) {
  // TODO: replace with real MPU-9250 reads.
  //   gyro[]  in deg/s, body frame
  //   accel[] in g,     body frame
  //   mag[]   in uT,    body frame (AK8963)
  // Return false if no new sample this call.
  return false;
}

bool readGPS() {
  // TODO: poll the u-blox. On a fresh solution, fill:
  //   S.fixType, S.numSats, S.hdop
  //   S.gpsSpeedMps, S.gpsCourseDeg, S.gpsCourseValid
  // The SparkFun library exposes getGroundSpeed() (mm/s) and getHeading()
  // (deg * 1e-5) from the PVT message. gpsCourseValid should require a 2D+ fix
  // AND speed above GPS_COURSE_MIN_MPS, since course is meaningless at rest.
  return false;
}

// ---------------------------------------------------------------------------
// Magnetometer -> tilt-compensated heading
// ---------------------------------------------------------------------------

float magHeading(const float mag[3], float pitchRad, float rollRad) {
  // Apply calibration
  float mx = (mag[0] - magCal.offset[0]) * magCal.scale[0];
  float my = (mag[1] - magCal.offset[1]) * magCal.scale[1];
  float mz = (mag[2] - magCal.offset[2]) * magCal.scale[2];

  // Tilt compensation using pitch/roll from the accelerometer
  float cosR = cosf(rollRad),  sinR = sinf(rollRad);
  float cosP = cosf(pitchRad), sinP = sinf(pitchRad);

  float xh = mx * cosP + mz * sinP;
  float yh = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

  float heading = atan2f(-yh, xh) * 57.29578f;
  heading += MAG_DECLINATION_DEG;   // true north
  return wrap360(heading);
}

// ---------------------------------------------------------------------------
// Fusion: complementary filter
//
//   gyro integrates the heading fast and smoothly but drifts.
//   the absolute reference (GPS course when moving, else mag) has no drift but
//   is noisy.
//   blend: heading = a*(heading + gyro*dt) + (1-a)*reference
//   with a = tau/(tau+dt).
//
// The sigma output is a heuristic, not a real covariance: it grows when the
// reference and the gyro-propagated heading disagree, and shrinks when they
// agree. Good enough for the HMI's engage gate; a real EKF would replace it.
// ---------------------------------------------------------------------------

void fuse(float dt) {
  float gyro[3], accel[3], mag[3];
  bool haveImu = readIMU(gyro, accel, mag);
  if (!haveImu) return;

  // Attitude from accelerometer (valid when not accelerating hard)
  S.accelMag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
  float pitchRad = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2]));
  float rollRad  = atan2f(accel[1], accel[2]);
  S.pitch = pitchRad * 57.29578f;
  S.roll  = rollRad  * 57.29578f;

  // Yaw rate straight off the gyro Z (body). Sign convention: positive = turn
  // to starboard. Flip if your mounting is inverted.
  S.yawRate = gyro[2];

  // Propagate heading with the gyro
  float propagated = wrap360(S.heading + S.yawRate * dt);

  // Choose the absolute reference
  float reference;
  bool haveRef = false;
  if (S.gpsCourseValid && S.gpsSpeedMps >= GPS_COURSE_MIN_MPS) {
    reference = S.gpsCourseDeg;
    haveRef = true;
    S.source = 3;            // FUSED (gyro + GPS)
    S.holding = false;
  } else if (magCal.valid && S.magCalQuality >= 2) {
    reference = magHeading(mag, pitchRad, rollRad);
    haveRef = true;
    S.source = 1;            // MAG
    S.holding = false;
  } else {
    // No trustworthy absolute reference. Coast on the gyro but let sigma grow
    // so the HMI knows confidence is decaying.
    S.source = (S.gpsSpeedMps >= GPS_COURSE_MIN_MPS) ? 3 : 0;
    S.holding = true;
  }

  if (haveRef) {
    float a = TAU / (TAU + dt);
    // Blend on the circle: nudge the propagated heading toward the reference
    float err = angDiff(reference, propagated);
    S.heading = wrap360(propagated + (1.0f - a) * err);

    // Sigma heuristic: track the disagreement between gyro and reference.
    float disagreement = fabsf(err);
    S.headingSigma = 0.9f * S.headingSigma + 0.1f * disagreement;
    if (S.headingSigma < 0.5f) S.headingSigma = 0.5f;
    S.valid = true;
  } else {
    S.heading = propagated;
    // Confidence decays while coasting
    S.headingSigma += dt * 2.0f;   // ~2 deg/s of growth with no reference
    if (S.headingSigma > 180) S.headingSigma = 180;
    S.valid = (S.headingSigma < 45.0f);  // give up past 45 deg
  }
}

// ---------------------------------------------------------------------------
// CAN transmit
// ---------------------------------------------------------------------------

void sendHeading() {
  CAN_message_t m;
  m.id = CAN_ID_HEADING;
  m.flags.extended = 1;
  m.len = 8;

  uint16_t h  = u16le_scaled(S.heading, 100.0f, 0.0f, 0, 35999);
  uint16_t sg = u16le_scaled(S.headingSigma, 100.0f, 0.0f, 0, 65535);
  uint16_t yr = u16le_scaled(S.yawRate, 100.0f, 32768.0f, 0, 65535);

  m.buf[0] = h & 0xFF;        m.buf[1] = h >> 8;
  m.buf[2] = sg & 0xFF;       m.buf[3] = sg >> 8;
  m.buf[4] = yr & 0xFF;       m.buf[5] = yr >> 8;
  m.buf[6] = S.source;
  m.buf[7] = (S.valid ? 0x01 : 0)
           | ((magCal.valid && S.magCalQuality >= 2) ? 0x02 : 0)
           | (S.gpsCourseValid ? 0x04 : 0)
           | (S.holding ? 0x08 : 0);
  can1.write(m);
}

void sendAttitude() {
  CAN_message_t m;
  m.id = CAN_ID_ATTITUDE;
  m.flags.extended = 1;
  m.len = 8;

  uint16_t p = u16le_scaled(S.pitch, 100.0f, 18000.0f, 0, 36000);
  uint16_t r = u16le_scaled(S.roll,  100.0f, 18000.0f, 0, 36000);
  uint16_t a = u16le_scaled(S.accelMag, 1000.0f, 0.0f, 0, 65535);

  m.buf[0] = p & 0xFF; m.buf[1] = p >> 8;
  m.buf[2] = r & 0xFF; m.buf[3] = r >> 8;
  m.buf[4] = a & 0xFF; m.buf[5] = a >> 8;
  m.buf[6] = 0;
  m.buf[7] = S.valid ? 0x01 : 0;
  can1.write(m);
}

void sendHealth() {
  CAN_message_t m;
  m.id = CAN_ID_HEALTH;
  m.flags.extended = 1;
  m.len = 8;

  uint16_t hd = u16le_scaled(S.hdop, 100.0f, 0.0f, 0, 65535);
  S.uptime = (uint16_t)((millis() - bootMillis) / 1000);

  m.buf[0] = S.fixType;
  m.buf[1] = S.numSats;
  m.buf[2] = hd & 0xFF; m.buf[3] = hd >> 8;
  m.buf[4] = S.magCalQuality;
  m.buf[5] = (uint8_t)S.tempC;
  m.buf[6] = S.uptime & 0xFF; m.buf[7] = S.uptime >> 8;
  can1.write(m);
}

// ---------------------------------------------------------------------------
// Magnetometer calibration routine
//
// Run this once (call from setup() with CALIBRATE defined, or trigger over
// serial). Rotate the boat slowly through all headings and orientations --
// figure-eights -- for ~60 s while the engine runs, so the hard-iron estimate
// captures the engine's field. Copy the printed offset/scale into magCal above
// and set valid = true.
// ---------------------------------------------------------------------------

void calibrateMag() {
  Serial.println("Mag calibration: rotate through all orientations for 60s...");
  float minv[3] = { 1e9,  1e9,  1e9};
  float maxv[3] = {-1e9, -1e9, -1e9};

  uint32_t start = millis();
  while (millis() - start < 60000) {
    float g[3], a[3], mg[3];
    if (readIMU(g, a, mg)) {
      for (int i = 0; i < 3; i++) {
        if (mg[i] < minv[i]) minv[i] = mg[i];
        if (mg[i] > maxv[i]) maxv[i] = mg[i];
      }
    }
    delay(10);
  }

  // Hard-iron offset = center of the min/max box.
  // Soft-iron scale  = normalize each axis to the average radius.
  float avgRadius = 0;
  float offset[3], radius[3];
  for (int i = 0; i < 3; i++) {
    offset[i] = (maxv[i] + minv[i]) / 2.0f;
    radius[i] = (maxv[i] - minv[i]) / 2.0f;
    avgRadius += radius[i];
  }
  avgRadius /= 3.0f;

  Serial.println("Paste into magCal, set valid = true:");
  Serial.print("  offset = {");
  Serial.print(offset[0]); Serial.print(", ");
  Serial.print(offset[1]); Serial.print(", ");
  Serial.print(offset[2]); Serial.println("},");
  Serial.print("  scale  = {");
  for (int i = 0; i < 3; i++) {
    float s = (radius[i] > 1e-3) ? avgRadius / radius[i] : 1.0f;
    Serial.print(s, 4);
    Serial.print(i < 2 ? ", " : "},\n");
  }
  Serial.println("  valid  = true");
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  bootMillis = millis();

  can1.begin();
  can1.setBaudRate(CAN_BITRATE);

  // MPU-9250 lives on Wire1 (SDA1=D17, SCL1=D16) with its own pullups.
  IMU_WIRE.begin();
  IMU_WIRE.setClock(400000);
  pinMode(PIN_MPU_INT, INPUT);

  // NEO-M9N: use the UART that the schematic breaks out (GPS_TX/GPS_RX on
  // Serial5). The primary I2C is shared with the ATECC608A, so keeping the
  // M9N on its UART avoids bus contention with the crypto chip.
  pinMode(PIN_GPS_RESET, OUTPUT);
  digitalWrite(PIN_GPS_RESET, HIGH);   // release reset (active low)
  pinMode(PIN_GPS_INT, INPUT);
  GPS_SERIAL.begin(38400);             // M9N default UART baud

  // TODO: init MPU-9250 on IMU_WIRE and the u-blox on GPS_SERIAL here.

  // To calibrate, uncomment:
  // calibrateMag();

  if (!magCal.valid) {
    Serial.println("WARNING: magnetometer not calibrated. Run calibrateMag(),");
    Serial.println("paste the results, and set magCal.valid = true. Until then");
    Serial.println("the node leans entirely on GPS course and cannot provide a");
    Serial.println("heading at rest.");
  }
}

void loop() {
  static uint32_t tImu = 0, tFuse = 0, tFast = 0, tHealth = 0;
  static uint32_t tPrevFuse = 0;
  uint32_t now = micros();

  // GPS is polled opportunistically; the library buffers UBX frames.
  readGPS();

  if (now - tFuse >= FUSE_PERIOD_US) {
    float dt = (tPrevFuse == 0) ? (FUSE_PERIOD_US / 1e6f)
                                : (now - tPrevFuse) / 1e6f;
    tPrevFuse = now;
    tFuse = now;
    fuse(dt);
  }

  if (now - tFast >= TX_FAST_US) {
    tFast = now;
    sendHeading();
    sendAttitude();
  }

  if (now - tHealth >= TX_HEALTH_US) {
    tHealth = now;
    sendHealth();
  }

  can1.events();
}
