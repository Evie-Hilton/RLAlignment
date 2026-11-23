/*
  StewartPlatformController.ino
  Arduino Nano ESP32 + PCA9685 + six rotary-servo Stewart platform.

  COMMAND API (one command per newline, case-sensitive):
    MAXP <positive float>  Set increment scale for translation, in mm.
    MAXA <positive float>  Set increment scale for rotation, in degrees.
    X  <amount>            Add amount * MAXP to X position.
    Y  <amount>            Add amount * MAXP to Y position.
    Z  <amount>            Add amount * MAXP to Z position.
    RX <amount>            Add amount * MAXA to roll.
    RY <amount>            Add amount * MAXA to pitch.
    RZ <amount>            Add amount * MAXA to yaw.
    h                      Return to calibrated neutral pose.
    l                      Laser OFF (active-low is NOT assumed; see setLaser()).
    L                      Laser ON.
    PING                   Reply OK.
    STATUS                 Return current target pose and controller status.

  Incremental semantics:
    RX 0.5 adds 0.5 * maxAngleDeg to the current roll target.
    RX 1.5 is accepted too. The API does not clamp the normalized amount.
    A requested pose is still rejected if inverse kinematics says it is
    unreachable or if it violates configured mechanical/servo limits.

  IMPORTANT CALIBRATION:
    This file contains a complete general rotary-horn IK solver, but it cannot
    move the assembled platform correctly until the geometry arrays below are
    filled with measurements from YOUR assembled platform and
    GEOMETRY_CALIBRATED is changed to true. Unknown coordinates are deliberately
    left as zeros rather than guessed.

  Geometry convention:
    - All geometry distances are millimetres.
    - Base frame origin: centre of base, +Z points upward.
    - Platform frame origin: centre of moving plate in neutral pose; axes are
      parallel to base axes in neutral pose.
    - HOME_Z_MM: vertical distance from base-frame origin to platform-frame
      origin at neutral.
    - BASE_PIVOT[i]: centre of servo's rotation axis in base coordinates.
    - SERVO_AXIS[i]: unit vector along positive servo rotation axis, in base
      coordinates, following the right-hand rule.
    - HORN_ZERO[i]: unit vector from servo pivot toward horn ball centre when
      the servo is at SERVO_CENTER_US[i]. It must be perpendicular to SERVO_AXIS.
    - PLATFORM_ANCHOR[i]: upper ball-joint centre in moving-platform coordinates.
    - HORN_LENGTH_MM[i]: servo-axis centre to lower horn ball-joint centre.
    - ROD_LENGTH_MM[i]: distance between the two ball-joint centres of the rod.
    - SERVO_DEG_PER_US[i]: measured angular change per microsecond of pulse.
    - SERVO_DIRECTION[i]: +1 if increasing pulse increases angle in the defined
      right-hand direction; -1 if it decreases it.
    - SERVO_MIN_ANGLE_DEG / MAX: safe angle change from neutral horn direction.

  Hardware safety:
    - Use an external, suitably rated servo supply and common ground.
    - Keep a physical power disconnect available during calibration.
    - Test with servo horns disconnected from the rods first.
    - Laser is initialized OFF. Verify your laser's TTL polarity before use.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// -----------------------------------------------------------------------------
// 1. Hardware configuration
// -----------------------------------------------------------------------------

Adafruit_PWMServoDriver pca9685(0x40);

constexpr uint8_t SERVO_COUNT = 6;
constexpr uint8_t SERVO_CHANNEL[SERVO_COUNT] = {0, 1, 2, 3, 4, 5};
constexpr uint8_t LASER_PIN = D7;  // Repository wiring: D7 / GPIO10 TTL enable.
constexpr float PCA_FREQUENCY_HZ = 50.0f;
constexpr uint16_t WIFI_PORT = 2323;
constexpr unsigned long WIFI_RETRY_MS = 15000;
constexpr unsigned long MOVE_DURATION_MS = 180;
constexpr unsigned long MOTION_UPDATE_MS = 10;
constexpr size_t COMMAND_BUFFER_SIZE = 80;

// Each input source has its own line buffer. Replies go back to the same source.
struct InputState {
  char buffer[COMMAND_BUFFER_SIZE];
  size_t length;
  bool discardingLongLine;
};

// Set these to your own network credentials. Leave SSID empty to disable Wi-Fi.
// Do NOT commit real credentials to a public Git repository.
const char WIFI_SSID[] = "";       // <-- ENTER Wi-Fi SSID, or leave empty to disable Wi-Fi.
const char WIFI_PASSWORD[] = "";   // <-- ENTER Wi-Fi password locally; do not commit it.


WiFiServer wifiServer(WIFI_PORT);
WiFiClient wifiClient;
bool wifiServerStarted = false;
unsigned long lastWiFiAttemptMs = 0;

// -----------------------------------------------------------------------------
// 2. MEASUREMENTS TO ENTER: geometry and servo calibration
// -----------------------------------------------------------------------------

// Change to true ONLY after filling in all geometry and servo calibration values.
constexpr bool GEOMETRY_CALIBRATED = true; // <-- CHANGE TO true after calibration.

// Measure the neutral distance between the base-frame origin and moving-platform
// origin. The base origin is the centre of the base; the platform origin is the
// centre of the moving plate. Use mm. Example only: do NOT guess this value.
const float HOME_Z_MM = 170.0f; // <-- MEASURE THIS.

// For each servo i=0..5, enter its axis centre in the BASE coordinate frame.
// Measure each rotation-axis centre (not the horn tip). Units: mm.
const float BASE_PIVOT[SERVO_COUNT][3] = {
  {84.0f, 0.0f, 0.0f}, // servo 0: <-- measure x,y,z
  {35.5f, -76.0f, 0.0f}, // servo 1: <-- measure x,y,z
  {-42.0f, -72.7f, 0.0f}, // servo 2: <-- measure x,y,z
  {-83.7f, 7.32f, 0.0f}, // servo 3: <-- measure x,y,z
  {-42.0f, 72.7f, 0.0f}, // servo 4: <-- measure x,y,z
  {48.0f, 69.0f, 0.0f}  // servo 5: <-- measure x,y,z
};

// Enter each servo's positive rotation-axis unit vector in BASE coordinates.
// These should be measured/determined from the servo mounting orientation.
const float SERVO_AXIS[SERVO_COUNT][3] = {
  {0.92f, -0.39f, 0.0f}, // servo 0: <-- ENTER unit vector
  {0.92f, -0.39f, 0.0f}, // servo 1: <-- ENTER unit vector
  {-0.80f, -0.60f, 0.0f}, // servo 4: <-- ENTER unit vector
  {-0.80f, -0.60f, 0.0f},  // servo 5: <-- ENTER unit vector
  {-0.12f, 0.99f, 0.0f}, // servo 2: <-- ENTER unit vector
  {-0.12f, 0.99f, 0.0f} // servo 3: <-- ENTER unit vector
};

// Enter the direction from each servo axis centre toward its horn ball centre
// when that servo is at its calibrated neutral pulse. Must be a unit vector
// perpendicular to SERVO_AXIS[i], expressed in BASE coordinates.
const float HORN_ZERO[SERVO_COUNT][3] = {
  {0.39f, 0.92f, 0.0f}, // servo 0: <-- ENTER unit vector
  {-0.39f, -0.92f, 0.0f}, // servo 1: <-- ENTER unit vector
  {0.6f, -0.91f, 0.0f}, // servo 2: <-- ENTER unit vector
  {-0.6f, 0.91f, 0.0f}, // servo 3: <-- ENTER unit vector
  {-0.99f, -0.12f, 0.0f}, // servo 4: <-- ENTER unit vector
  {0.99f, 0.12f, 0.0f}  // servo 5: <-- ENTER unit vector
};

// Upper ball-joint coordinates relative to the centre of the moving platform,
// with the platform in its neutral coordinate frame. Units: mm.
const float PLATFORM_ANCHOR[SERVO_COUNT][3] = {
  {51.5f, 0.0f, 0.0f}, // servo/rod 0: <-- measure x,y,z
  {47.0f, -20.9f, 0.0f}, // servo/rod 1: <-- measure x,y,z
  {-25.75f, -44.6f, 0.0f}, // servo/rod 2: <-- measure x,y,z
  {-41.7f, -30.3, 0.0f}, // servo/rod 3: <-- measure x,y,z
  {-5.4f, 51.2f, 0.0f}, // servo/rod 4: <-- measure x,y,z
  {-25.75f, 44.6f, 0.0f}  // servo/rod 5: <-- measure x,y,z
};

// Servo electrical calibration. Centre pulse should correspond to the neutral
// pose, and HORN_ZERO[i] should point along the horn at this pulse.
// These nominal defaults are only starting points; calibrate every servo.
uint16_t SERVO_CENTER_US[SERVO_COUNT] = {1470, 1500, 1620, 1550, 1560, 1520}; 
uint16_t SERVO_MIN_US[SERVO_COUNT]    = {600, 600, 600, 600, 600, 600};       // <-- verify safe limits
uint16_t SERVO_MAX_US[SERVO_COUNT]    = {2400, 2400, 2400, 2400, 2400, 2400}; // <-- verify safe limits
int8_t SERVO_DIRECTION[SERVO_COUNT]   = {+1, -1, +1, -1, +1, -1};             

// Measure angle change per microsecond around the centre pulse for each servo.
// A nominal 180-degree servo across 2000 us is about 0.09 deg/us, but do not
// rely on that estimate for final calibration.
float SERVO_DEG_PER_US[SERVO_COUNT] = {0.085f, 0.09f, 0.086f, 0.087f, 0.086f, 0.084f}; // <-- measure

// Safe angle change relative to neutral horn direction. Set conservative values
// from the real mechanism; the solver rejects poses outside these bounds.
float SERVO_MIN_ANGLE_DEG[SERVO_COUNT] = {-45, -45, -45, -45, -45, -45}; // <-- calibrate
float SERVO_MAX_ANGLE_DEG[SERVO_COUNT] = { 45,  45,  45,  45,  45,  45}; // <-- calibrate



// actually needed calibration:
// the length of each horn. this is the shortest distance between the center of the ball joint
// and the rotation axis of the servo.
const float HORN_LENGTH_MM[SERVO_COUNT] = {
  24.0f, // servo 0: <-- measure
  24.0f, // servo 1: <-- measure
  24.0f, // servo 2: <-- measure
  24.0f, // servo 3: <-- measure
  24.0f, // servo 4: <-- measure
  24.0f  // servo 5: <-- measure
};

// The length of each rod. The distance between the centers of each ball joint.
const float ROD_LENGTH_MM[SERVO_COUNT] = {
  180.0f, // rod 0: <-- measure
  180.0f, // rod 1: <-- measure
  180.0f, // rod 2: <-- measure
  180.0f, // rod 3: <-- measure
  180.0f, // rod 4: <-- measure
  180.0f  // rod 5: <-- measure
};

// the positive angle between the x-axis and the direction each horn is facing in degrees
const float HORN_Z[SERVO_COUNT] = {
  270.0f,
  90.0f,
  120.0f,
  300.0f,
  60.0f,
  240.0f
};

struct Vec3 { float x, y, z; };
struct Pose {
  float x, y, z;       // translations in mm relative to neutral
  float rx, ry, rz;    // roll, pitch, yaw in degrees, Euler convention Rz*Ry*Rx
};
Vec3 vec(float x, float y, float z) { return {x, y, z}; }

// assuming a platform frame with (0, 0, 0) as the center of the points, where is each point on the platform?
const Vec3 PLATFORM_HOME[SERVO_COUNT] = {
  vec(0.0f - 50.7343f, 0.0f - 11.9145f, 0.0f),
  vec(0.0f - 50.7343f, 22.27f - 11.9145f, 0.0f),
  vec(66.3861f - 50.7343f, 58.4187f - 11.9145f, 0.0f),
  vec(84.7578f - 50.7343f, 50.6723f - 11.9145f, 0.0f),
  vec(85.65f - 50.7343f, -24.73f - 11.9145f, 0.0f),
  vec(67.612f - 50.7343f, -35.144f - 11.9145f, 0.0f)
};

// Given the platform is in the home position, each vector in this array represents lk = Pk - Bk from the inverse
// kinematics formulation i'm using: https://raw.org/research/inverse-kinematics-of-a-stewart-platform/.
const Vec3 HOME_ROD_VECTORS[SERVO_COUNT] = {
  vec(28.2323f, 26.41384f, 170.676f),
  vec(28.2323f, -26.41384f, 170.676f),
  vec(8.7589f, -37.6568f, 170.676f),
  vec(-36.9912f, -11.243f, 170.676f),
  vec(-36.9912f, 11.243f, 170.676f),
  vec(8.7589f, 37.6568f, 170.676f)
};

// -----------------------------------------------------------------------------
// 3. Pose state and motion state
// -----------------------------------------------------------------------------

Pose targetPose = {0, 0, 0, 0, 0, 0};
float currentServoAngleDeg[SERVO_COUNT] = {0, 0, 0, 0, 0, 0}; // angle relative to neutral
float currentPulseUs[SERVO_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};
float moveStartPulseUs[SERVO_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};
float moveTargetPulseUs[SERVO_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};
bool motionActive = false;
unsigned long motionStartMs = 0;
unsigned long lastMotionUpdateMs = 0;

InputState serialInput = {{0}, 0, false};
InputState wifiInput = {{0}, 0, false};

// Increment scale values. X/Y/Z increments are MAXP millimetres; RX/RY/RZ
// increments are MAXA degrees. Defaults are intentionally small.
float maxPositionMm = 1.0f;
float maxAngleDeg = 1.0f;
bool laserIsOn = false;

// -----------------------------------------------------------------------------
// 4. Vector and rotation math
// -----------------------------------------------------------------------------

Vec3 add(Vec3 a, Vec3 b) { return vec(a.x+b.x, a.y+b.y, a.z+b.z); }
Vec3 sub(Vec3 a, Vec3 b) { return vec(a.x-b.x, a.y-b.y, a.z-b.z); }
Vec3 mul(Vec3 a, float s) { return vec(a.x*s, a.y*s, a.z*s); }
float dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
  return vec(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
float norm(Vec3 a) { return sqrtf(dot(a, a)); }
bool normalize(Vec3 &a) {
  float n = norm(a);
  if (!isfinite(n) || n < 1e-6f) return false;
  a = mul(a, 1.0f/n);
  return true;
}
Vec3 readVec3(const float a[SERVO_COUNT][3], uint8_t i) {
  return vec(a[i][0], a[i][1], a[i][2]);
}
float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// Rotate a platform-local point by Rz(yaw) * Ry(pitch) * Rx(roll).
Vec3 rotatePlatformPoint(Vec3 p, const Pose &pose) {
  const float rx = pose.rx * DEG_TO_RAD;
  const float ry = pose.ry * DEG_TO_RAD;
  const float rz = pose.rz * DEG_TO_RAD;
  const float cx = cosf(rx), sx = sinf(rx);
  const float cy = cosf(ry), sy = sinf(ry);
  const float cz = cosf(rz), sz = sinf(rz);

  // Rx
  Vec3 q = vec(p.x,
               cx*p.y - sx*p.z,
               sx*p.y + cx*p.z);
  // Ry
  q = vec(cy*q.x + sy*q.z,
          q.y,
          -sy*q.x + cy*q.z);
  // Rz
  q = vec(cz*q.x - sz*q.y,
          sz*q.x + cz*q.y,
          q.z);
  return q;
}

// -----------------------------------------------------------------------------
// 5. Rotary-servo inverse kinematics
// -----------------------------------------------------------------------------

/*
  Given the list of vectors 'rod_vectors', calculate the angles each servo should be at. The
  angle formulation and vector definitions come from the inverse kinematics of a stewart platform
  as defined by: https://raw.org/research/inverse-kinematics-of-a-stewart-platform/.
  'rod_vectors' is an array of vectors which represents each lk = Pk - Bk from the formulation
*/
bool calculateInverseKinematics(const Vec3 (&rod_vectors)[SERVO_COUNT], float anglesOut[SERVO_COUNT]){
  // no update is made if the geometry hasn't been calibrated
  if (!GEOMETRY_CALIBRATED) return false;

  float angles[SERVO_COUNT];

  // loop over each servo and calculate its new angle
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    const float h = HORN_LENGTH_MM[i];
    const float h2 = h*h;
    const Vec3& l = rod_vectors[i];
    const float l2 = l.x*l.x + l.y*l.y + l.z*l.z;
    const float b = HORN_Z[i] * DEG_TO_RAD;
    const float d2 = ROD_LENGTH_MM[i] * ROD_LENGTH_MM[i];

    const float ek = 2 * h * l.z;
    const float fk = 2 * h * (cosf(b) * l.x + sinf(b) * l.y);
    const float gk = l2 - (d2 - h2);

    if (!isfinite(ek*ek + fk*fk) || ek*ek + fk*fk <= 1e-6f) {
        return false;
    }
    const float arg = gk/sqrtf(ek*ek + fk*fk);
    if (!isfinite(arg) || fabsf(arg) > 1.0f) return false;
    const float angle = asin(arg) - atan2(fk, ek);

    // update servo angle
    angles[i] = angle * RAD_TO_DEG;
  }
  for (uint8_t i = 0; i < SERVO_COUNT; ++i){
    anglesOut[i] = angles[i];
  }
  return true; // if all servo angles are able to be calculated, return true to indicate the angles have been calculated
}

// Given a Pose, calculate the 'rod_vectors' array as defined in calculateInverseKinematics().
// Everything will be done relative to the home rod_vectors array 'HOME_ROD_VECTORS', which is calculated above.
void calculate_rod_vectors(const Pose &pose, Vec3 rod_vectors[SERVO_COUNT]) {
  // Calculate each rod vector relative to the calibrated home position.
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    // Rotate each platform point by the pose rotation.
    Vec3 newPoint = rotatePlatformPoint(PLATFORM_HOME[i], pose);

    // Translate each platform point by the pose translation.
    newPoint = add(newPoint, vec(pose.x, pose.y, pose.z));

    // Calculate the offset from the home position and apply it
    // to the corresponding home rod vector.
    const Vec3 offset = sub(newPoint, PLATFORM_HOME[i]);
    rod_vectors[i] = add(HOME_ROD_VECTORS[i], offset);
  }
}

// -----------------------------------------------------------------------------
// 6. Servo output and smooth non-blocking movement
// -----------------------------------------------------------------------------

void writeServoPulse(uint8_t i, float pulseUs) {
  pulseUs = clampFloat(pulseUs, (float)SERVO_MIN_US[i], (float)SERVO_MAX_US[i]);
  currentPulseUs[i] = pulseUs;
  pca9685.writeMicroseconds(SERVO_CHANNEL[i], (uint16_t)lroundf(pulseUs));
}

bool anglesToPulses(const float angles[SERVO_COUNT], float pulsesOut[SERVO_COUNT]) {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    if (angles[i] < SERVO_MIN_ANGLE_DEG[i] || angles[i] > SERVO_MAX_ANGLE_DEG[i]) return false;
    float pulse = (float)SERVO_CENTER_US[i]
                  + (float)SERVO_DIRECTION[i] * angles[i] / SERVO_DEG_PER_US[i];
    if (!isfinite(pulse) || pulse < SERVO_MIN_US[i] || pulse > SERVO_MAX_US[i]) return false;
    pulsesOut[i] = pulse;
  }
  return true;
}

void scheduleServoMove(const float angles[SERVO_COUNT], const float pulses[SERVO_COUNT]) {
  // currentPulseUs contains the last interpolated output because updateMotion()
  // is serviced frequently in loop(). A new command can safely retarget in flight.
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    moveStartPulseUs[i] = currentPulseUs[i];
    moveTargetPulseUs[i] = pulses[i];
    currentServoAngleDeg[i] = angles[i]; // last commanded branch, not sensor feedback
  }
  motionStartMs = millis();
  lastMotionUpdateMs = 0;
  motionActive = true;
}

void updateMotion() {
  if (!motionActive) return;
  unsigned long now = millis();
  if (now - lastMotionUpdateMs < MOTION_UPDATE_MS && now - motionStartMs < MOVE_DURATION_MS) return;
  lastMotionUpdateMs = now;

  float fraction = (float)(now - motionStartMs) / (float)MOVE_DURATION_MS;
  if (fraction >= 1.0f) fraction = 1.0f;
  // Smoothstep interpolation: zero velocity at start and end.
  float smooth = fraction * fraction * (3.0f - 2.0f*fraction);
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    float pulse = moveStartPulseUs[i] + (moveTargetPulseUs[i] - moveStartPulseUs[i]) * smooth;
    writeServoPulse(i, pulse);
  }
  if (fraction >= 1.0f) motionActive = false;
}

bool tryApplyPose(const Pose &candidate, Stream &reply) {
  float angles[SERVO_COUNT];
  float pulses[SERVO_COUNT];

  Vec3 rod_vectors[SERVO_COUNT];
  calculate_rod_vectors(candidate, rod_vectors);

  if (!calculateInverseKinematics(rod_vectors, angles)) {
    reply.println("ERR IK_UNREACHABLE_OR_UNCALIBRATED");
    return false;
  }
  if (!anglesToPulses(angles, pulses)) {
    reply.println("ERR SERVO_LIMIT");
    return false;
  }
  targetPose = candidate;
  scheduleServoMove(angles, pulses);
  reply.println("OK");
  return true;
}

// Add an incremental command. Values are deliberately NOT clamped to [-1, 1].
bool applyIncrement(char axisName, float amount, Stream &reply) {
  Pose candidate = targetPose;
  switch (axisName) {
    case 'X':  candidate.x  += amount * maxPositionMm; break;
    case 'Y':  candidate.y  += amount * maxPositionMm; break;
    case 'Z':  candidate.z  += amount * maxPositionMm; break;
    case 'A':  candidate.rx += amount * maxAngleDeg; break; // RX
    case 'B':  candidate.ry += amount * maxAngleDeg; break; // RY
    case 'C':  candidate.rz += amount * maxAngleDeg; break; // RZ
    default: reply.println("ERR INTERNAL_AXIS"); return false;
  }
  return tryApplyPose(candidate, reply); // Keeps old target if IK fails.
}

// -----------------------------------------------------------------------------
// 7. Laser, status, and command parsing
// -----------------------------------------------------------------------------

void setLaser(bool on) {
  laserIsOn = on;
  digitalWrite(LASER_PIN, on ? HIGH : LOW); // Verify polarity for your laser module.
}

void sendStatus(Stream &reply) {
  reply.print("STATUS X="); reply.print(targetPose.x, 4);
  reply.print(" Y="); reply.print(targetPose.y, 4);
  reply.print(" Z="); reply.print(targetPose.z, 4);
  reply.print(" RX="); reply.print(targetPose.rx, 4);
  reply.print(" RY="); reply.print(targetPose.ry, 4);
  reply.print(" RZ="); reply.print(targetPose.rz, 4);
  reply.print(" MAXP="); reply.print(maxPositionMm, 4);
  reply.print(" MAXA="); reply.print(maxAngleDeg, 4);
  reply.print(" LASER="); reply.print(laserIsOn ? "ON" : "OFF");
  reply.print(" MOTION="); reply.println(motionActive ? "MOVING" : "IDLE");
}

bool parseOneFloat(char *text, float &value) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

void handleCommand(char *line, Stream &reply) {
  char *savePtr = nullptr;
  char *name = strtok_r(line, " \t", &savePtr);
  char *arg = strtok_r(nullptr, " \t", &savePtr);
  char *extra = strtok_r(nullptr, " \t", &savePtr);
  if (name == nullptr) return;
  if (extra != nullptr) { reply.println("ERR TOO_MANY_ARGUMENTS"); return; }

  if (strcmp(name, "PING") == 0) {
    if (arg) { reply.println("ERR ARGUMENT_COUNT"); return; }
    reply.println("OK");
    return;
  }
  if (strcmp(name, "STATUS") == 0) {
    if (arg) { reply.println("ERR ARGUMENT_COUNT"); return; }
    sendStatus(reply);
    return;
  }
  if (strcmp(name, "h") == 0) {
    if (arg) { reply.println("ERR ARGUMENT_COUNT"); return; }
    Pose home = {0, 0, 0, 0, 0, 0};
    tryApplyPose(home, reply);
    return;
  }
  if (strcmp(name, "l") == 0 || strcmp(name, "L") == 0) {
    if (arg) { reply.println("ERR ARGUMENT_COUNT"); return; }
    setLaser(name[0] == 'L'); // Per requested API: lowercase off, uppercase on.
    reply.println("OK");
    return;
  }

  if (arg == nullptr) { reply.println("ERR MISSING_ARGUMENT"); return; }
  float value;
  if (!parseOneFloat(arg, value)) { reply.println("ERR INVALID_NUMBER"); return; }

  if (strcmp(name, "MAXP") == 0) {
    if (value <= 0.0f) { reply.println("ERR MAXP_MUST_BE_POSITIVE"); return; }
    maxPositionMm = value;
    reply.println("OK");
    return;
  }
  if (strcmp(name, "MAXA") == 0) {
    if (value <= 0.0f) { reply.println("ERR MAXA_MUST_BE_POSITIVE"); return; }
    maxAngleDeg = value;
    reply.println("OK");
    return;
  }

  // API is case-sensitive. Translation commands are X/Y/Z; rotation commands RX/RY/RZ.
  if (strcmp(name, "X") == 0)  { applyIncrement('X', value, reply); return; }
  if (strcmp(name, "Y") == 0)  { applyIncrement('Y', value, reply); return; }
  if (strcmp(name, "Z") == 0)  { applyIncrement('Z', value, reply); return; }
  if (strcmp(name, "RX") == 0) { applyIncrement('A', value, reply); return; }
  if (strcmp(name, "RY") == 0) { applyIncrement('B', value, reply); return; }
  if (strcmp(name, "RZ") == 0) { applyIncrement('C', value, reply); return; }

  reply.print("ERR UNKNOWN_COMMAND ");
  reply.println(name);
}

// -----------------------------------------------------------------------------
// 8. Line-oriented USB and Wi-Fi input
// -----------------------------------------------------------------------------

void serviceInput(Stream &source, InputState &state) {
  while (source.available() > 0) {
    char c = (char)source.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (state.discardingLongLine) {
        state.discardingLongLine = false;
        state.length = 0;
        source.println("ERR COMMAND_TOO_LONG");
        continue;
      }
      state.buffer[state.length] = '\0';
      handleCommand(state.buffer, source);
      state.length = 0;
      continue;
    }
    if (state.discardingLongLine) continue;
    if (state.length < COMMAND_BUFFER_SIZE - 1) {
      state.buffer[state.length++] = c;
    } else {
      state.length = 0;
      state.discardingLongLine = true;
    }
  }
}

void serviceWiFi() {
  if (WIFI_SSID[0] == '\0') return; // Wi-Fi disabled until credentials are entered.

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiClient && !wifiClient.connected()) wifiClient.stop();
    wifiServerStarted = false;
    if (millis() - lastWiFiAttemptMs >= WIFI_RETRY_MS) {
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      lastWiFiAttemptMs = millis();
    }
    return;
  }

  if (!wifiServerStarted) {
    wifiServer.begin();
    wifiServerStarted = true;
    Serial.print("Wi-Fi console ready at ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(WIFI_PORT);
  }

  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient incoming = wifiServer.available();
    if (incoming) {
      wifiClient = incoming;
      wifiClient.setNoDelay(true);
      wifiInput.length = 0;
      wifiInput.discardingLongLine = false;
      wifiClient.println("READY");
    }
  }
  if (wifiClient && wifiClient.connected()) serviceInput(wifiClient, wifiInput);
}

// -----------------------------------------------------------------------------
// 9. Arduino initialization and main loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LASER_PIN, OUTPUT);
  setLaser(false); // Always boot with laser OFF.

  // Nano ESP32 board mapping used by the repository: SDA=A4, SCL=A5.
  Wire.begin(A4, A5);
  pca9685.begin();
  pca9685.setOscillatorFrequency(25000000);
  pca9685.setPWMFreq(PCA_FREQUENCY_HZ);
  delay(100);

  // Initialize the driver to the measured neutral pulses. Ensure the servo
  // supply is current-limited and the mechanism is safe before powering it.
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    currentPulseUs[i] = SERVO_CENTER_US[i];
    moveStartPulseUs[i] = SERVO_CENTER_US[i];
    moveTargetPulseUs[i] = SERVO_CENTER_US[i];
    pca9685.writeMicroseconds(SERVO_CHANNEL[i], SERVO_CENTER_US[i]);
  }

  Serial.println("READY");
  Serial.println("Commands: MAXP n | MAXA n | X/Y/Z n | RX/RY/RZ n | h | l | L | PING | STATUS");
  Serial.println("Amounts are incremental multipliers; they are not clamped to [-1,1].");
  if (!GEOMETRY_CALIBRATED) {
    Serial.println("WARNING: IK geometry not calibrated. Pose commands will be rejected until measurements are entered and GEOMETRY_CALIBRATED=true.");
  }

  if (WIFI_SSID[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWiFiAttemptMs = millis();
    Serial.print("Connecting to Wi-Fi; MAC=");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("Wi-Fi disabled: enter credentials in WIFI_SSID/WIFI_PASSWORD to enable TCP port 2323.");
  }
}

void loop() {
  updateMotion();
  serviceInput(Serial, serialInput);
  serviceWiFi();
  updateMotion();
}