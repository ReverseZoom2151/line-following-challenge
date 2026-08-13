// Host tests for navigator.h.
//
// The navigator is driven with hand-built snapshots and an explicit clock, so
// a whole run of the course can be played out in a few microseconds without a
// robot, and each transition asserted individually.

#include "arduino_stub.h"
#include "test_harness.h"

#include "navigator.h"

// Builds a snapshot directly from normalised values. raw is set to match so
// the same helper can also feed calibration.
static SensorSnapshot snap(uint16_t a, uint16_t b, uint16_t c, uint16_t d,
                           uint16_t e) {
  SensorSnapshot s = {};
  const uint16_t v[NUM_SENSORS] = { a, b, c, d, e };
  for (int i = 0; i < NUM_SENSORS; i++) {
    s.normalised[i] = v[i];
    s.raw[i] = v[i];
    s.timedOut[i] = false;
  }
  return s;
}

static const SensorSnapshot BLANK = snap(0, 0, 0, 0, 0);
static const SensorSnapshot CENTRED = snap(0, 100, 900, 100, 0);
static const SensorSnapshot LEFT_BRANCH = snap(900, 300, 700, 0, 0);
static const SensorSnapshot RIGHT_BRANCH = snap(0, 0, 700, 300, 900);
static const SensorSnapshot CROSS = snap(900, 800, 900, 800, 900);

// Calibration needs every sensor to see both surfaces, so these sweep all
// five between a light and a dark reading.
static const SensorSnapshot CAL_LIGHT = snap(200, 200, 200, 200, 200);
static const SensorSnapshot CAL_DARK = snap(1200, 1200, 1200, 1200, 1200);

// Sweeps until calibration finishes, and stops the moment it does.
static uint32_t runCalibration(Navigator_c &nav, uint32_t t) {
  for (int i = 0; i < 200 && nav.state() == NavState::Calibrate; i++) {
    t += 100;
    nav.update(t, (i % 2) ? CAL_DARK : CAL_LIGHT);
  }
  return t;
}

// Runs the machine forward to the point where it has left Calibrate and
// JoinLine and is following the line, and returns the clock it reached.
static uint32_t driveToFollowLine(Navigator_c &nav, uint32_t t) {

  t = runCalibration(nav, t);

  // join: two debounced sightings of the line
  for (int i = 0; i < 10 && nav.state() != NavState::FollowLine; i++) {
    t += 250;
    nav.update(t, CENTRED);
  }

  return t;
}

int main() {
  printf("navigator\n");

  {
    TEST("begin starts in Calibrate and leaves it once the sweep is done");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    CHECK(nav.state() == NavState::Calibrate);

    uint32_t t = 0;
    for (int i = 0; i < 5; i++) {
      t += 100;
      nav.update(t, CAL_LIGHT);
    }
    CHECK(nav.state() == NavState::Calibrate);  // still sweeping
    CHECK(t < CALIBRATION_MS);

    t = runCalibration(nav, t);

    CHECK(t >= CALIBRATION_MS);
    CHECK(nav.state() == NavState::JoinLine);
    CHECK(sensors.isCalibrated());
  }

  {
    TEST("JoinLine needs two debounced sightings before it follows");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    uint32_t t = runCalibration(nav, 0);
    CHECK(nav.state() == NavState::JoinLine);

    // repeated sightings inside the debounce window count once
    t += 10;
    nav.update(t, CENTRED);
    t += 10;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::JoinLine);

    t += 300;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::FollowLine);
  }

  {
    TEST("FollowLine steers forward and stays put on a centred line");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);
    CHECK(nav.state() == NavState::FollowLine);

    t += 10;
    nav.update(t, CENTRED);

    CHECK(nav.state() == NavState::FollowLine);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);
    // a centred line means no steering correction, so both wheels match
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, mockPins[R_PWM_PIN].analogValue);
    CHECK(mockPins[L_PWM_PIN].analogValue > 0);
  }

  {
    TEST("FollowLine steers towards a line that has drifted off centre");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    // line under DN2: left of centre, without reaching the far sensor
    t += 10;
    nav.update(t, snap(0, 900, 300, 0, 0));
    CHECK(nav.state() == NavState::FollowLine);
    int leftDrift = mockPins[L_PWM_PIN].analogValue;
    int rightDrift = mockPins[R_PWM_PIN].analogValue;

    // mirrored: line under DN4
    t += 10;
    nav.update(t, snap(0, 0, 300, 900, 0));
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, rightDrift);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, leftDrift);
    CHECK(leftDrift != rightDrift);
  }

  {
    // DEFECT: the turn states spin with a blocking pause, so update() does
    // not return for 250ms and the robot is blind for the whole turn. The
    // clock jumping inside a single update is the proof.
    TEST("DEFECT: a far sensor turn blocks for the duration of the spin");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    t += 10;
    nav.update(t, LEFT_BRANCH);
    CHECK(nav.state() == NavState::TurnLeft);

    uint32_t before = micros();
    t += 10;
    nav.update(t, LEFT_BRANCH);
    CHECK_EQ(micros() - before, 250000);
    CHECK(nav.state() == NavState::FollowLine);

    t += 10;
    nav.update(t, RIGHT_BRANCH);
    CHECK(nav.state() == NavState::TurnRight);

    before = micros();
    t += 10;
    nav.update(t, RIGHT_BRANCH);
    CHECK_EQ(micros() - before, 250000);
    CHECK(nav.state() == NavState::FollowLine);
  }

  {
    TEST("both far sensors mean a crossroads, and it clears on the far side");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);
    t += 10;
    nav.update(t, CROSS);
    CHECK(nav.state() == NavState::Crossroads);

    // still over the junction: keeps going straight
    t += 10;
    nav.update(t, CROSS);
    CHECK(nav.state() == NavState::Crossroads);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);

    t += 10;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::FollowLine);
  }

  {
    TEST("losing the line enters Rediscover, and finding it again leaves");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    t += 10;
    nav.update(t, BLANK);
    CHECK(nav.state() == NavState::Rediscover);

    t += 10;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::FollowLine);
  }

  {
    TEST("Rediscover gives up into Halted, which is idempotent");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.initialise();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    t += 10;
    nav.update(t, BLANK);
    CHECK(nav.state() == NavState::Rediscover);

    t += REDISCOVER_TIMEOUT_MS + 1;
    nav.update(t, BLANK);
    CHECK(nav.state() == NavState::Halted);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 0);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 0);

    // Halted stays halted, whatever the sensors say, and returns each time
    for (int i = 0; i < 20; i++) {
      t += 10;
      nav.update(t, (i % 2) ? CENTRED : CROSS);
      CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 0);
      CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 0);
    }
    CHECK(nav.state() == NavState::Halted);
  }

  return testSummary("navigator");
}
