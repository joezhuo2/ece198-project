/*
 * ece 198 - wearable exercise tracker, minimal version (arduino uno r4 minima)
 *
 * worn on the hip, guides the user through a cccare session (warm-up, cardio,
 * strength, cool-down), counts reps in the strength phase and warns about
 * leaning or dropping too fast
 *
 * hardware:
 *   hc-sr04 ultrasonic  - trig d7, echo d8, points at the floor
 *   bmx160 imu          - i2c (a4 sda / a5 scl), 3.3v
 *   ssd1306 oled 128x64 - i2c (same bus, address 0x3c)
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobot_BMX160.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);
DFRobot_BMX160 imu;

const int TRIG = 7;
const int ECHO = 8;

// timing and filtering
const unsigned long PERIOD_MS = 100;  // 10 samples per second
const float SMOOTH = 0.3;             // 0-1, lower = smoother but slower

// movement and safety limits
const float STILL_SPEED = 5.0;        // cm/s, slower than this counts as still
const float MIN_REP_DEPTH = 8.0;      // cm of hip travel needed for a rep
const float TILT_LIMIT = 15.0;        // degrees from the standing pose
const float SPEED_LIMIT = 50.0;       // cm/s of downward hip movement
const unsigned long SET_REST_MS = 20000;  // still this long = new set

// sensor health checks
const float MIN_DIST = 2.0, MAX_DIST = 400.0;  // hc-sr04 valid range in cm
const float MIN_G = 4.9, MAX_G = 19.6;         // m/s^2, gravity is ~9.8
const int FAIL_SAMPLES = 10;                   // 1 s of bad readings = failure
const int FROZEN_SAMPLES = 20;                 // 2 s of identical imu data = failure

// session plan from the dosage document
struct Phase {
  const char *name;
  const char *hint;
  unsigned long minutes;
};

const Phase PHASES[] = {
  {"WARM-UP", "Low intensity", 5},
  {"CARDIO", "40-60% HR reserve", 20},
  {"STRENGTH", "1-2 sets x 10-20", 30},
  {"COOL-DOWN", "Low intensity", 5},
};
const int PHASE_COUNT = sizeof(PHASES) / sizeof(PHASES[0]);
const int STRENGTH_PHASE = 2;

// gravity direction while standing, measured at startup
float refX, refY, refZ;

// state carried between loops
unsigned long sessionStart, lastTime, lastMoveTime;
float dist = 0, tilt = 0;
bool distValid = false;
int distFails = 0, imuFails = 0, imuFrozen = 0;
float lastImu[6];
int reps = 0, sets = 1;
bool goingDown = false;
float topDist, bottomDist;

// shows up to four lines of text on a cleared screen
void showMessage(const char *a, const char *b = "", const char *c = "", const char *d = "") {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(a);
  display.println(b);
  display.println(c);
  display.println(d);
  display.display();
}

// returns distance in cm, or 0 if the echo timed out or is out of range
float readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  // sound travels 0.034 cm/us, halved for the round trip, 30 ms timeout so a
  // missed echo doesn't block the loop
  float d = pulseIn(ECHO, HIGH, 30000) * 0.034 / 2;
  return (d >= MIN_DIST && d <= MAX_DIST) ? d : 0;
}

// averages 1 s of accelerometer data while the user stands still, returns
// false if they moved or the readings don't look like gravity
bool calibrate() {
  float sx = 0, sy = 0, sz = 0, lo = 1e9, hi = 0;
  for (int i = 0; i < 20; i++) {
    sBmx160SensorData_t m, g, a;
    imu.getAllData(&m, &g, &a);
    float mag = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    lo = min(lo, mag);
    hi = max(hi, mag);
    sx += a.x;
    sy += a.y;
    sz += a.z;
    delay(50);
  }
  float norm = sqrt(sx * sx + sy * sy + sz * sz);
  float avgG = norm / 20;
  if (hi - lo > 1.0 || avgG < MIN_G || avgG > MAX_G) return false;
  refX = sx / norm;
  refY = sy / norm;
  refZ = sz / norm;
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // without a screen there is nowhere to show errors, so report over usb
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (true) {}
  }
  display.setTextColor(SSD1306_WHITE);

  if (!imu.begin()) {
    showMessage("IMU not found!", "Check A4/A5 wiring,", "then press RESET");
    while (true) {}
  }

  while (true) {
    showMessage("Stand upright", "and still...");
    delay(1500);
    if (calibrate()) break;
    showMessage("Moved or IMU error", "Retrying...");
    delay(1500);
  }

  for (int i = 3; i > 0; i--) {
    char line[16];
    snprintf(line, sizeof(line), "Starting in %d", i);
    showMessage("Calibrated!", line, "", "RESET = restart");
    delay(1000);
  }

  dist = readDistance();
  distValid = dist > 0;
  
  sessionStart = lastTime = lastMoveTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // distance: smooth good readings, keep the last value on a bad one, and only
  // compute velocity (positive = moving down) between two good readings
  float raw = readDistance();
  float prevDist = dist;
  float velocity = 0;
  if (raw > 0) {
    dist = distValid ? dist + SMOOTH * (raw - dist) : raw;
    if (distValid) velocity = (prevDist - dist) / dt;
    distValid = true;
    distFails = 0;
  } else {
    distValid = false;
    distFails = min(distFails + 1, FAIL_SAMPLES);
  }
  bool distError = distFails >= FAIL_SAMPLES;

  // imu: a working sensor is never perfectly constant, so identical readings
  // mean it stopped responding, a magnitude far from 1 g means bad data
  sBmx160SensorData_t m, g, a;
  imu.getAllData(&m, &g, &a);
  float reading[6] = {a.x, a.y, a.z, g.x, g.y, g.z};
  bool frozen = memcmp(reading, lastImu, sizeof(reading)) == 0;
  imuFrozen = frozen ? min(imuFrozen + 1, FROZEN_SAMPLES) : 0;
  memcpy(lastImu, reading, sizeof(reading));
  float mag = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  bool magOk = mag >= MIN_G && mag <= MAX_G;
  imuFails = magOk ? 0 : min(imuFails + 1, FAIL_SAMPLES);
  bool imuError = imuFails >= FAIL_SAMPLES || imuFrozen >= FROZEN_SAMPLES;

  // tilt is the angle between gravity now and gravity while standing
  if (magOk && !imuError) {
    float cosTilt = (a.x * refX + a.y * refY + a.z * refZ) / mag;
    float rawTilt = acos(constrain(cosTilt, -1.0f, 1.0f)) * 180.0 / PI;
    tilt += SMOOTH * (rawTilt - tilt);
  }

  // find the current phase and the seconds left in it
  unsigned long elapsed = (now - sessionStart) / 1000;
  int phase = 0;
  while (phase < PHASE_COUNT && elapsed >= PHASES[phase].minutes * 60) {
    elapsed -= PHASES[phase].minutes * 60;
    phase++;
  }
  bool strength = phase == STRENGTH_PHASE;

  // rep = down then up with enough hip travel, a long rest starts a new set
  if (fabs(velocity) > STILL_SPEED) lastMoveTime = now;
  if (!strength || distError) {
    goingDown = false;
  } else {
    if (velocity > STILL_SPEED && !goingDown) {
      goingDown = true;
      topDist = prevDist;
      bottomDist = dist;
    }
    if (goingDown) bottomDist = min(bottomDist, dist);
    if (velocity < -STILL_SPEED && goingDown) {
      if (topDist - bottomDist >= MIN_REP_DEPTH) reps++;
      goingDown = false;
    }
    if (reps > 0 && now - lastMoveTime > SET_REST_MS) {
      sets++;
      reps = 0;
    }
  }

  // one warning at a time, sensor failures first since nothing else is trustworthy
  const char *warning = "";
  if (imuError) warning = "!! IMU ERROR !!";
  else if (distError) warning = "!! DIST SENSOR ERR !!";
  else if (tilt > TILT_LIMIT) warning = "!! EXCESSIVE LEAN !!";
  else if (velocity > SPEED_LIMIT) warning = "!! TOO FAST !!";

  // screen: phase and timer, hint, warning, reps, live readings
  display.clearDisplay();
  display.setCursor(0, 0);
  if (phase >= PHASE_COUNT) {
    display.println("SESSION DONE");
    display.println("RESET = new session");
  } else {
    unsigned long left = PHASES[phase].minutes * 60 - elapsed;
    char line[22];
    snprintf(line, sizeof(line), "%-10s %02lu:%02lu", PHASES[phase].name, left / 60, left % 60);
    display.println(line);
    display.println(PHASES[phase].hint);
  }

  display.setCursor(0, 20);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.println(warning);
  display.setTextColor(SSD1306_WHITE);

  if (strength) {
    display.setCursor(0, 34);
    display.print("Set ");
    display.print(sets);
    display.print("  Reps ");
    display.print(reps);
    display.println("/10-20");
  }

  display.drawFastHLine(0, 44, 128, SSD1306_WHITE);
  display.setCursor(0, 47);
  display.print("Tilt ");
  if (imuError) display.print("--");
  else display.print(tilt, 0);
  display.print("  Dist ");
  if (distError) display.print("--");
  else display.print(dist, 0);
  display.setCursor(0, 56);
  if (distError) display.println("Moving: ?");
  else if (velocity > STILL_SPEED) display.println("Moving: DOWN");
  else if (velocity < -STILL_SPEED) display.println("Moving: UP");
  else display.println("Moving: still");
  display.display();

  // hold a steady 100 ms sample period
  while (millis() - now < PERIOD_MS) {}
}
