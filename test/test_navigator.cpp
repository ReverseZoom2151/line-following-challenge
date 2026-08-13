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
// A corner: the far sensor is on the line and the way ahead is not.
static const SensorSnapshot LEFT_BRANCH = snap(900, 400, 300, 0, 0);
static const SensorSnapshot RIGHT_BRANCH = snap(0, 0, 300, 400, 900);
// A side mark passed at speed: the far sensor fires but the line runs on.
static const SensorSnapshot LEFT_SIDE_MARK = snap(900, 300, 900, 300, 0);
static const SensorSnapshot RIGHT_SIDE_MARK = snap(0, 300, 900, 300, 900);
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

// Presents the same view twice, which is what the navigator requires before
// it acts on a junction.
static uint32_t confirm(Navigator_c &nav, uint32_t t, const SensorSnapshot &s) {
  t += 10;
  nav.update(t, s);
  t += 10;
  nav.update(t, s);
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
    motors.begin();
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
    motors.begin();
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
    motors.begin();
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
    motors.begin();
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
    // A turn must not block: update() returns at once and the turn is ticked
    // across many iterations, so the sensors are read throughout it.
    TEST("a turn is ticked, not blocked on, and exits on reacquisition");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.begin();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    t = confirm(nav, t, LEFT_BRANCH);
    CHECK(nav.state() == NavState::TurnLeft);
    uint32_t turn_started = t;

    // spinning left, with no time lost inside update()
    uint32_t before = micros();
    t += 10;
    nav.update(t, BLANK);
    CHECK_EQ(micros() - before, 0);
    CHECK(nav.state() == NavState::TurnLeft);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, REV);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);

    // the line it started on cannot end the turn straight away
    t += 10;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::TurnLeft);

    // past the settle window, the line coming back ends the turn at once,
    // well before the timeout would have
    t += TURN_SETTLE_MS;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::FollowLine);
    CHECK((t - turn_started) < TURN_TIMEOUT_MS);

    // and the same on the other side
    t = confirm(nav, t, RIGHT_BRANCH);
    CHECK(nav.state() == NavState::TurnRight);
    t += 10;
    nav.update(t, BLANK);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, REV);
    t += TURN_SETTLE_MS;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::FollowLine);
  }

  {
    TEST("a turn that never reacquires the line exits on the timeout");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.begin();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    t = confirm(nav, t, LEFT_BRANCH);
    CHECK(nav.state() == NavState::TurnLeft);

    uint32_t turn_started = t;

    // spins and spins, finding nothing
    for (int i = 0; i < 100 && nav.state() == NavState::TurnLeft; i++) {
      t += 20;
      nav.update(t, BLANK);
    }

    CHECK(nav.state() != NavState::TurnLeft);
    CHECK(nav.state() == NavState::Rediscover);
    CHECK((t - turn_started) >= TURN_TIMEOUT_MS);
    // it gave up on the timeout rather than spinning indefinitely
    CHECK((t - turn_started) < TURN_TIMEOUT_MS + 100);
  }

  {
    TEST("both far sensors mean a crossroads, and it clears on the far side");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.begin();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);
    t = confirm(nav, t, CROSS);
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
    motors.begin();
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
    motors.begin();
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

  {
    TEST("a single noisy frame on a far sensor is not a junction");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.begin();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    // one frame of far-left, then the line again
    t += 10;
    nav.update(t, LEFT_BRANCH);
    CHECK(nav.state() == NavState::FollowLine);
    t += 10;
    nav.update(t, CENTRED);
    CHECK(nav.state() == NavState::FollowLine);

    // the same on the right, and at a crossroads
    t += 10;
    nav.update(t, RIGHT_BRANCH);
    CHECK(nav.state() == NavState::FollowLine);
    t += 10;
    nav.update(t, CENTRED);
    t += 10;
    nav.update(t, CROSS);
    CHECK(nav.state() == NavState::FollowLine);
  }

  {
    TEST("a side mark with the line running on is not a corner");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    sensors.begin();
    motors.begin();
    nav.begin(&sensors, &motors);

    uint32_t t = driveToFollowLine(nav, 0);

    // held for several frames: still not a turn, because the way ahead is
    // clearly the line the robot is already following
    for (int i = 0; i < 5; i++) {
      t += 10;
      nav.update(t, LEFT_SIDE_MARK);
    }
    CHECK(nav.state() == NavState::FollowLine);

    for (int i = 0; i < 5; i++) {
      t += 10;
      nav.update(t, RIGHT_SIDE_MARK);
    }
    CHECK(nav.state() == NavState::FollowLine);

    // whereas the same far sensor with nothing ahead is a corner
    confirm(nav, t, LEFT_BRANCH);
    CHECK(nav.state() == NavState::TurnLeft);
  }

  {
    TEST("classification is symmetric between left and right");
    mockReset();
    LineSensor_c sensorsL;
    Motors_c motorsL;
    Navigator_c navL;
    sensorsL.begin();
    motorsL.begin();
    navL.begin(&sensorsL, &motorsL);
    uint32_t tl = driveToFollowLine(navL, 0);
    tl = confirm(navL, tl, LEFT_BRANCH);

    mockReset();
    LineSensor_c sensorsR;
    Motors_c motorsR;
    Navigator_c navR;
    sensorsR.begin();
    motorsR.begin();
    navR.begin(&sensorsR, &motorsR);
    uint32_t tr = driveToFollowLine(navR, 0);
    tr = confirm(navR, tr, RIGHT_BRANCH);

    // mirrored evidence, mirrored decision, taken at the same moment: the
    // old rules required the line present to turn left and absent to turn
    // right, so the two sides could never agree
    CHECK(navL.state() == NavState::TurnLeft);
    CHECK(navR.state() == NavState::TurnRight);
    CHECK_EQ(tl, tr);
  }

  {
    TEST("beginCalibrated skips the sweep and keeps the restored bounds");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    motors.begin();
    sensors.begin();

    uint16_t lo[NUM_SENSORS];
    uint16_t hi[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { lo[i] = 150; hi[i] = 2000; }
    CHECK(sensors.applyCalibration(lo, hi, NUM_SENSORS));

    nav.beginCalibrated(&sensors, &motors);

    // straight to looking for the line, no sweep
    CHECK(nav.state() == NavState::JoinLine);

    // and the restored bounds survived. begin() would have reopened a sweep
    // and thrown them away on the first snapshot, which is the whole reason
    // this does not route through it.
    CHECK(sensors.isCalibrated());
    CHECK_EQ(sensors.calibrationMin(0), 150);
    CHECK_EQ(sensors.calibrationMax(0), 2000);
  }

  {
    TEST("begin still opens a sweep and starts in Calibrate");
    mockReset();
    LineSensor_c sensors;
    Motors_c motors;
    Navigator_c nav;
    motors.begin();
    sensors.begin();

    uint16_t lo[NUM_SENSORS];
    uint16_t hi[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { lo[i] = 150; hi[i] = 2000; }
    CHECK(sensors.applyCalibration(lo, hi, NUM_SENSORS));

    nav.begin(&sensors, &motors);

    CHECK(nav.state() == NavState::Calibrate);
    CHECK(!sensors.isCalibrated());  // the sweep discarded the old bounds
  }

  return testSummary("navigator");
}
