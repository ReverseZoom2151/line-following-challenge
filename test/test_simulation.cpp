// Closed-loop host simulation of the robot on a course.
//
// The other host tests feed the navigator snapshots by hand. That proves the
// state machine transitions correctly, but it cannot prove the robot goes
// anywhere: nothing in a hand-built snapshot depends on what the motors were
// told to do a moment earlier. This file closes that loop.
//
// Each iteration:
//   1. the track and the current pose decide what the five sensors see
//   2. those discharge times are pushed into the virtual pins
//   3. the REAL LineSensor_c::readAll() produces the snapshot
//   4. the REAL Navigator_c::update() decides what to do
//   5. the commanded PWM is read back out of the virtual motor pins
//   6. the kinematics model moves the robot
//
// Nothing in the firmware is stubbed or reimplemented. What IS invented is the
// world: the motor response, the sensor response and the course geometry, all
// of which live in kinematics.h and track.h and are described there as models.
//
// The limits of the result are worth stating plainly, because a green run here
// is easy to mistake for evidence about the robot:
//
//   - Wheel slip does not exist here. A commanded turn always produces exactly
//     the modelled rotation.
//   - Motors are linear above a fixed deadband, identical to each other, and
//     respond instantly with no inertia.
//   - Sensors are noiseless, identical, and see a hard-edged stripe.
//   - Battery voltage never sags, so the PWM-to-speed map never moves.
//
// Everything the simulation demonstrates is therefore a statement about the
// firmware's LOGIC under a model, and never a statement about the hardware.

#include "arduino_stub.h"
#include "test_harness.h"

#include "kinematics.h"
#include "track.h"

#include "navigator.h"

using sim::Pose;
using sim::Track;

// The control period. The firmware's main loop is free-running, so this is a
// choice, not a measurement. 5 ms is a plausible rate for a loop whose only
// blocking cost is the sensor read.
static const uint32_t CONTROL_PERIOD_US = 5000;

// ------------------------------------------------------------- the runner

struct Sim {

  LineSensor_c sensors;
  Motors_c motors;
  Navigator_c nav;

  Track track;
  Pose pose;

  uint32_t iterations = 0;

  // How far ahead of the wheel axle the sensor bar is modelled as sitting.
  // Defaults to the nominal figure; a test can shorten it to see how much of
  // the loop's stability depends on that guess.
  double sensorLead = sim::SENSOR_FORWARD_OFFSET_MM;

  // Recorded so a test can assert on the shape of a run rather than only on
  // where it ended up.
  double lastLeftPwm = 0.0;
  double lastRightPwm = 0.0;

  void begin(const Track &t, const Pose &start) {

    mockReset();
    track = t;
    pose = start;
    iterations = 0;

    sensors.begin();
    motors.begin();
    nav.begin(&sensors, &motors);

  }

  // Reads a signed PWM back off the virtual pins, the way an oscilloscope on
  // the motor driver would see it.
  static double commandedPwm(uint8_t pwmPin, uint8_t dirPin) {
    const double magnitude = (double)mockPins[pwmPin].analogValue;
    return (mockPins[dirPin].digitalValue == FWD) ? magnitude : -magnitude;
  }

  // One sensor read at the current pose, through the firmware's own readAll().
  SensorSnapshot sense() {

    sim::applyToMockPins(
        sim::readTrack(track, pose, sim::BLACK_DISCHARGE_US, sensorLead));

    SensorSnapshot s;
    sensors.readAll(s);

    // The stub's watchdog counts reads across the whole process, and a long
    // run makes many millions of them. It exists to catch a single read loop
    // that never terminates, so the budget is restored once per control
    // iteration; a genuine hang still trips it inside one iteration.
    mockDigitalReadCount = 0;

    return s;

  }

  void tick() {

    const uint32_t started_us = mockNowMicros;

    SensorSnapshot s = sense();

    // The sensor read consumed virtual time. Pad out to a fixed period so the
    // control rate does not silently vary with what the sensors are seeing.
    if (mockNowMicros < started_us + CONTROL_PERIOD_US) {
      mockNowMicros = started_us + CONTROL_PERIOD_US;
    }

    nav.update(millis(), s);

    lastLeftPwm = commandedPwm(L_PWM_PIN, L_DIR_PIN);
    lastRightPwm = commandedPwm(R_PWM_PIN, R_DIR_PIN);

    pose = sim::step(pose, lastLeftPwm, lastRightPwm,
                     (double)CONTROL_PERIOD_US * 1e-6);

    iterations++;

  }

  double crossTrackError() const {
    return track.distanceToCentreline(pose.x, pose.y);
  }

  // Runs until the predicate holds or the budget is spent. Returns the number
  // of iterations actually taken.
  template <typename Pred>
  uint32_t runUntil(uint32_t budget, Pred done) {
    uint32_t n = 0;
    while (n < budget && !done(*this)) {
      tick();
      n++;
    }
    return n;
  }

  uint32_t runFor(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) tick();
    return n;
  }

};

// ---------------------------------------------------------------- helpers

static const char *stateName(NavState s) {
  switch (s) {
    case NavState::Calibrate:  return "Calibrate";
    case NavState::JoinLine:   return "JoinLine";
    case NavState::FollowLine: return "FollowLine";
    case NavState::TurnLeft:   return "TurnLeft";
    case NavState::TurnRight:  return "TurnRight";
    case NavState::Crossroads: return "Crossroads";
    case NavState::Rediscover: return "Rediscover";
    case NavState::Halted:     return "Halted";
  }
  return "?";
}

// Runs the firmware's own calibration sweep to completion, then puts the robot
// back where it started.
//
// The reposition is a deliberate cheat, and it is worth being explicit about
// why. Calibrate spins the robot on the spot for CALIBRATION_MS and then hands
// straight over to JoinLine, so where the robot is pointing when following
// begins is entirely a function of the modelled spin rate, which is a guess.
// Testing corner geometry against that guess would be testing the guess. The
// heading the sweep actually leaves behind is asserted on separately below,
// where it is the subject rather than a nuisance.
static void calibrateThenReset(Sim &s, const Pose &restart) {

  s.runUntil(4000, [](const Sim &x) { return x.nav.state() != NavState::Calibrate; });
  s.pose = restart;

}

// Drives out of JoinLine and into FollowLine, or gives up.
static bool joinLine(Sim &s, uint32_t budget = 400) {

  s.runUntil(budget, [](const Sim &x) {
    return x.nav.state() == NavState::FollowLine || x.nav.state() == NavState::Halted;
  });

  return s.nav.state() == NavState::FollowLine;

}

// Calibrate, reposition, join. The common opening for most scenarios.
static bool startOnLine(Sim &s, const Track &t, const Pose &start) {

  s.begin(t, start);
  calibrateThenReset(s, start);

  // Asserted here rather than left to chance: a scenario that quietly ran with
  // a rejected calibration would be testing the fallback path while claiming
  // to test line following, and the fallback path behaves very differently.
  CHECK(s.sensors.isCalibrated());

  return joinLine(s);

}

int main() {
  printf("simulation\n");

  // ------------------------------------------------------------------ model
  {
    TEST("the motor and kinematics model is self-consistent");

    // Deadband: nothing moves at a demand the motor cannot break stiction at.
    CHECK_NEAR(sim::wheelSpeed(5.0), 0.0, 1e-9);
    CHECK(sim::wheelSpeed(30.0) > 0.0);
    CHECK_NEAR(sim::wheelSpeed(-30.0), -sim::wheelSpeed(30.0), 1e-9);

    // Straight ahead stays straight, and a symmetric spin stays put.
    Pose p;
    p = sim::step(p, 30.0, 30.0, 1.0);
    CHECK_NEAR(p.y, 0.0, 1e-9);
    CHECK_NEAR(p.heading, 0.0, 1e-9);
    CHECK_NEAR(p.x, sim::wheelSpeed(30.0), 1e-9);

    Pose q;
    q = sim::step(q, -30.0, 30.0, 0.5);
    CHECK_NEAR(q.x, 0.0, 1e-9);
    CHECK_NEAR(q.y, 0.0, 1e-9);
    CHECK(q.heading > 0.0);  // positive rate is a left turn

    printf("    model: base PWM %.0f gives %.1f mm/s per wheel, "
           "spin rate %.2f rad/s\n",
           (double)BASE_SPEED_PWM, sim::wheelSpeed(BASE_SPEED_PWM),
           sim::bodyRate(-BASE_SPEED_PWM, BASE_SPEED_PWM));
  }

  // ----------------------------------------------------------- sensor model
  {
    TEST("the sensor model puts the line where the firmware thinks it is");

    Sim s;
    Track t = sim::makeStraight();
    s.begin(t, Pose{0.0, 0.0, 0.0});

    // Centred on the line: the middle sensor is darkest and the far pair see
    // nothing, since the line is 19 mm wide and they sit 18 mm off centre.
    sim::SensorReadings r = sim::readTrack(t, Pose{0.0, 0.0, 0.0});
    CHECK(r.us[2] > r.us[1]);
    CHECK(r.us[2] > r.us[3]);
    CHECK_EQ(r.us[0], sim::WHITE_DISCHARGE_US);
    CHECK_EQ(r.us[4], sim::WHITE_DISCHARGE_US);
    CHECK(r.us[2] < SENSOR_TIMEOUT_US);

    // Shifted left of the line, the right-hand sensors darken.
    sim::SensorReadings shifted = sim::readTrack(t, Pose{0.0, 12.0, 0.0});
    CHECK(shifted.us[3] > shifted.us[1]);
  }

  // -------------------------------------------------- calibration behaviour
  {
    TEST("the calibration sweep produces a usable calibration");

    Sim s;
    s.begin(sim::makeStraight(), Pose{100.0, 0.0, 0.0});
    s.runUntil(4000, [](const Sim &x) { return x.nav.state() != NavState::Calibrate; });

    CHECK(s.nav.state() == NavState::JoinLine);
    CHECK(s.sensors.isCalibrated());

    for (int i = 0; i < NUM_SENSORS; i++) {
      const uint16_t span = s.sensors.calibrationMax(i) - s.sensors.calibrationMin(i);
      CHECK(span >= MIN_CALIBRATION_SPAN);
    }

    printf("    calibration spans (us):");
    for (int i = 0; i < NUM_SENSORS; i++) {
      printf(" %u", (unsigned)(s.sensors.calibrationMax(i) - s.sensors.calibrationMin(i)));
    }
    printf("\n");

    // Where the sweep leaves the robot pointing. Reported here, and asserted
    // on in the opening-sequence scenario further down.
    printf("    after the sweep: heading %.1f deg, pose (%.1f, %.1f)\n",
           s.pose.heading * 180.0 / sim::PI, s.pose.x, s.pose.y);
  }

  // ------------------------------------------- the rejected calibration path
  {
    // This scenario is a defect report written as a test. Every assertion in
    // it records what the firmware does, not what it should do.
    TEST("a rejected calibration leaves the robot unable to tell floor from line");

    Sim s;
    // The line begins exactly where the robot does. The calibration sweep
    // turns one way only, so the sensors on that side spend the whole sweep
    // over bare floor and never learn what black looks like.
    Track t = sim::makeStraight(600.0, 0.0);
    s.begin(t, Pose{0.0, 0.0, 0.0});
    s.runUntil(4000, [](const Sim &x) { return x.nav.state() != NavState::Calibrate; });

    printf("    rejected calibration: spans");
    for (int i = 0; i < NUM_SENSORS; i++) {
      printf(" %u", (unsigned)(s.sensors.calibrationMax(i) - s.sensors.calibrationMin(i)));
    }
    printf(", calibrated %d\n", (int)s.sensors.isCalibrated());

    // endCalibration() correctly refuses a calibration where a sensor never
    // saw both surfaces. That guard works.
    CHECK(!s.sensors.isCalibrated());

    // What happens next does not. With no calibration, normaliseOne() falls
    // back to the range 0..SENSOR_TIMEOUT_US, so bare floor discharging in
    // about 180 us normalises to roughly 72 on every sensor. Five of those sum
    // to about 360, and linePosition() calls anything above
    // LINE_PRESENT_THRESHOLD (200) a line. The robot therefore reports a
    // perfectly centred line while sitting on blank floor.
    s.track = sim::makeBlank();
    s.pose = Pose{0.0, 0.0, 0.0};

    SensorSnapshot blank = s.sense();

    int32_t total = 0;
    for (int i = 0; i < NUM_SENSORS; i++) total += blank.normalised[i];

    bool found = false;
    const float pos = s.sensors.linePosition(blank, found);

    printf("    on blank floor: normalised total %d against a threshold of %u, "
           "lineFound %d, position %.3f\n",
           (int)total, (unsigned)LINE_PRESENT_THRESHOLD, (int)found, (double)pos);

    // The sum still clears the threshold, which is exactly why testing the
    // sum was wrong. What matters is that linePosition() no longer believes
    // it: the presence test is on the strongest single sensor now, so blank
    // floor reports no line.
    CHECK(total > (int32_t)LINE_PRESENT_THRESHOLD);
    CHECK(!found);
    CHECK_NEAR(pos, 0.0, 0.01);  // and invents no steering demand

    // onLine() was never fooled, because it tests each sensor individually.
    // The two now agree, which is the point: a silent disagreement between
    // them was what let the robot drive on blank floor believing it was
    // centred on a line.
    CHECK(!s.sensors.onLine(blank));

    // The consequence, now bounded. JoinLine used to have no timeout of any
    // kind, unlike every other moving state, so the robot drove straight for
    // as long as it had power. It now gives up after JOIN_TIMEOUT_MS and
    // halts with the motors stopped.
    const double x0 = s.pose.x;
    s.runFor(3000);  // 15 simulated seconds

    printf("    after 15 s on blank floor: state %s, travelled %.0f mm\n",
           stateName(s.nav.state()), s.pose.x - x0);

    CHECK(s.nav.state() == NavState::Halted);
    CHECK(mockPins[L_PWM_PIN].analogValue == 0);
    CHECK(mockPins[R_PWM_PIN].analogValue == 0);

    // It must have stopped near where the timeout expired rather than having
    // run on. At BASE_SPEED_PWM the model covers well under 500 mm in the
    // 5 s the join is allowed.
    CHECK((s.pose.x - x0) < 500.0);
  }

  // ------------------------------------------------- the uncheated opening
  {
    // Every other scenario repositions the robot after the calibration sweep.
    // This one does not, and runs the opening exactly as the firmware would on
    // a course: calibrate, then join, from a robot placed on the line.
    TEST("the opening sequence, run without repositioning the robot");

    Sim s;
    Track t = sim::makeStraight(2000.0);
    s.begin(t, Pose{100.0, 0.0, 0.0});

    s.runUntil(4000, [](const Sim &x) { return x.nav.state() != NavState::Calibrate; });
    const double sweptTo = s.pose.heading;

    s.runUntil(3000, [](const Sim &x) {
      return x.nav.state() == NavState::FollowLine || x.nav.state() == NavState::Halted;
    });

    printf("    opening: sweep left it at %.0f deg, then %s at (%.0f, %.0f)\n",
           sweptTo * 180.0 / sim::PI, stateName(s.nav.state()),
           s.pose.x, s.pose.y);

    // The finding, asserted rather than described. Calibrate spins the robot
    // on the spot for CALIBRATION_MS and hands straight over to JoinLine with
    // no attempt to square up to the line first, so the heading it starts
    // following from is whatever the sweep happened to end on. Under this
    // model that is most of a half turn.
    CHECK(std::fabs(sweptTo) > 1.0);

    // What happens next is now bounded either way. Driving off at that
    // heading, the robot either crosses the line and follows it, or fails to
    // find it and halts when JOIN_TIMEOUT_MS expires. Both are acceptable;
    // driving away indefinitely, which is what it used to do, is not.
    CHECK(s.nav.state() == NavState::FollowLine ||
          s.nav.state() == NavState::Halted);

    // The underlying gap is still open and cannot be closed from here: the
    // firmware has no way to know which way it is pointing. Squaring up after
    // the sweep needs either the encoders or the gyro, neither of which is
    // wired in, and a timed counter-spin would only be as good as the guessed
    // spin rate. Until then, place the robot so that the line runs through it
    // rather than starting at it, and expect the opening heading to be
    // whatever the sweep ended on.
    if (s.nav.state() == NavState::Halted) {
      CHECK(mockPins[L_PWM_PIN].analogValue == 0);
    }
  }

  // ---------------------------------------------------- stability margin
  {
    // The straight-line result above is comfortably damped, and it is worth
    // knowing what is doing the damping. The sensor bar leads the wheel axle,
    // so it reports where the robot is about to be rather than where it is,
    // which is a derivative term supplied by geometry rather than by the PID.
    // That lead is a guessed number. This sweep shortens it and reports what
    // happens to the loop at the shipped PID_KP of 40.
    TEST("stability at PID_KP 40 depends on the modelled sensor lead");

    const double leads[] = { 40.0, 25.0, 15.0, 8.0 };
    double lateHalfPeak[4] = {};
    int crossingCount[4] = {};
    bool heldTheLine[4] = {};

    for (int k = 0; k < 4; k++) {

      Sim s;
      s.sensorLead = leads[k];
      Track t = sim::makeStraight(6000.0);
      s.begin(t, Pose{0.0, 8.0, 0.0});
      calibrateThenReset(s, Pose{0.0, 8.0, 0.0});
      joinLine(s);

      double prev = s.pose.y;
      const int N = 1600;
      int ran = 0;

      for (int i = 0; i < N && s.nav.state() == NavState::FollowLine; i++) {
        s.tick();
        ran++;
        const double y = s.pose.y;
        if ((y > 0.0) != (prev > 0.0)) crossingCount[k]++;
        prev = y;
        if (i >= N / 2 && std::fabs(y) > lateHalfPeak[k]) lateHalfPeak[k] = std::fabs(y);
      }

      heldTheLine[k] = (ran == N) && (s.nav.state() == NavState::FollowLine);

      printf("    lead %4.0f mm: %2d crossings, late amplitude %5.2f mm, "
             "held the line %d, ended %s\n",
             leads[k], crossingCount[k], lateHalfPeak[k],
             (int)heldTheLine[k], stateName(s.nav.state()));

    }

    // At the nominal lead the transient is gone entirely and there is no
    // sustained oscillation to speak of.
    CHECK(lateHalfPeak[0] < 1.0);
    CHECK(heldTheLine[0]);
    CHECK_EQ(crossingCount[0], 0);

    // Shortening the lead does cost damping: the settled amplitude at 15 mm is
    // two orders of magnitude larger than at 40 mm.
    CHECK(lateHalfPeak[2] > lateHalfPeak[0]);

    // But the loop still holds the line at every lead tried, all the way down
    // to 8 mm, and the amplitude stays small in absolute terms. That is the
    // honest result, and it is a stronger one than the assertion originally
    // written here expected: the stability of PID_KP 40 in this model does not
    // rest on the single most uncertain constant in the geometry, because
    // varying it fivefold does not break it.
    //
    // What does the stabilising is the low base speed. At BASE_SPEED_PWM 30
    // the modelled robot travels about 41 mm/s, so it has a long time to
    // correct a small error. The steering authority is enormous by comparison:
    // PID_KP 40 against a base of 30 means the inner wheel is commanded into
    // reverse once the line passes three quarters of full deflection. A robot
    // that actually ran faster would be a different control problem, and this
    // file has nothing to say about it.
    for (int k = 0; k < 4; k++) {
      CHECK(heldTheLine[k]);
      CHECK(lateHalfPeak[k] < 5.0);
    }
  }

  // --------------------------------------------------------------- straight
  {
    TEST("it converges onto a straight line from an offset start");

    Sim s;
    Track t = sim::makeStraight();
    CHECK(startOnLine(s, t, Pose{0.0, 7.0, 0.0}));

    double worst = 0.0;
    for (int i = 0; i < 1200 && s.nav.state() == NavState::FollowLine; i++) {
      s.tick();
      const double e = s.crossTrackError();
      if (e > worst) worst = e;
    }

    printf("    straight: state %s, x %.0f mm, worst error %.1f mm, "
           "final error %.1f mm\n",
           stateName(s.nav.state()), s.pose.x, worst, s.crossTrackError());

    CHECK(s.nav.state() == NavState::FollowLine);
    CHECK(s.pose.x > 100.0);            // it actually went somewhere
    CHECK(worst < 20.0);                // never left the line
    CHECK(s.crossTrackError() < 6.0);   // and settled onto it
  }

  // ------------------------------------------------------------ oscillation
  {
    TEST("it does not oscillate unboundedly at the shipped PID_KP");

    Sim s;
    Track t = sim::makeStraight(4000.0);
    CHECK(startOnLine(s, t, Pose{0.0, 8.0, 0.0}));

    // Signed cross-track error, so a swing across the line is visible.
    double firstHalfPeak = 0.0;
    double secondHalfPeak = 0.0;
    int crossings = 0;
    double prev = s.pose.y;

    const int N = 1600;
    for (int i = 0; i < N && s.nav.state() == NavState::FollowLine; i++) {
      s.tick();
      const double y = s.pose.y;
      if ((y > 0.0) != (prev > 0.0)) crossings++;
      prev = y;
      const double mag = std::fabs(y);
      if (i < N / 2) { if (mag > firstHalfPeak) firstHalfPeak = mag; }
      else { if (mag > secondHalfPeak) secondHalfPeak = mag; }
    }

    printf("    oscillation: %d zero crossings, first-half peak %.2f mm, "
           "second-half peak %.2f mm\n",
           crossings, firstHalfPeak, secondHalfPeak);

    CHECK(s.nav.state() == NavState::FollowLine);
    // The amplitude must not grow. Equality is allowed: a steady limit cycle
    // is bounded, and bounded is what is claimed here.
    CHECK(secondHalfPeak <= firstHalfPeak + 0.5);
    CHECK(secondHalfPeak < 15.0);
  }

  // ----------------------------------------------------------------- curve
  {
    TEST("it tracks a gentle curve");

    Sim s;
    Track t = sim::makeGentleCurve(300.0, 300.0);
    CHECK(startOnLine(s, t, Pose{0.0, 0.0, 0.0}));

    double worst = 0.0;
    int followed = 0;
    for (int i = 0; i < 8000; i++) {
      s.tick();
      if (s.nav.state() != NavState::FollowLine) break;
      followed++;
      const double e = s.crossTrackError();
      if (e > worst) worst = e;
    }

    printf("    curve: %d iterations following, ended at (%.0f, %.0f) "
           "heading %.0f deg, worst error %.1f mm, state %s\n",
           followed, s.pose.x, s.pose.y, s.pose.heading * 180.0 / sim::PI,
           worst, stateName(s.nav.state()));

    // Round the quarter circle: the exit runs along +y, so a heading near
    // +90 degrees means the curve was actually followed rather than cut. The
    // arc ends at (600, 300), after which the line simply stops, so leaving
    // FollowLine at that point is the course ending and not a failure. The
    // axle stops about one sensor lead short of the arc end, because it is the
    // sensors that run out of line first.
    CHECK(worst < 20.0);
    CHECK(s.pose.heading > 1.3);
    CHECK(s.pose.y > 250.0);
    CHECK(s.pose.x > 550.0);
  }

  // ------------------------------------------------------------- 90 degrees
  {
    TEST("it gets round a square left-hand corner and back onto the line");

    Sim s;
    Track t = sim::makeCorner90(500.0, 500.0);
    CHECK(startOnLine(s, t, Pose{0.0, 0.0, 0.0}));

    bool sawTurn = false;
    bool sawRediscover = false;

    for (int i = 0; i < 6000; i++) {
      s.tick();
      if (s.nav.state() == NavState::TurnLeft || s.nav.state() == NavState::TurnRight) {
        sawTurn = true;
      }
      if (s.nav.state() == NavState::Rediscover) sawRediscover = true;
      if (s.nav.state() == NavState::Halted) break;
      if (s.pose.y > 300.0) break;
    }

    printf("    corner: ended at (%.0f, %.0f) heading %.0f deg, state %s, "
           "error %.1f mm, turn seen %d, rediscover seen %d\n",
           s.pose.x, s.pose.y, s.pose.heading * 180.0 / sim::PI,
           stateName(s.nav.state()), s.crossTrackError(),
           (int)sawTurn, (int)sawRediscover);

    CHECK(s.pose.y > 300.0);                    // it went up the exit leg
    CHECK(s.crossTrackError() < 20.0);          // and is on the line there
    CHECK(s.nav.state() != NavState::Halted);
  }

  // ------------------------------------------------------------- crossroads
  {
    TEST("it drives across a crossroads and carries on");

    Sim s;
    Track t = sim::makeCrossroads(400.0, 900.0, 200.0);
    CHECK(startOnLine(s, t, Pose{0.0, 0.0, 0.0}));

    bool sawCross = false;
    bool sawTurn = false;

    for (int i = 0; i < 4000; i++) {
      s.tick();
      if (s.nav.state() == NavState::Crossroads) sawCross = true;
      if (s.nav.state() == NavState::TurnLeft || s.nav.state() == NavState::TurnRight) {
        sawTurn = true;
      }
      if (s.pose.x > 700.0 || s.nav.state() == NavState::Halted) break;
    }

    printf("    crossroads: ended at (%.0f, %.0f), state %s, error %.1f mm, "
           "cross seen %d, turn seen %d\n",
           s.pose.x, s.pose.y, stateName(s.nav.state()), s.crossTrackError(),
           (int)sawCross, (int)sawTurn);

    CHECK(sawCross);                     // recognised as a junction, not a corner
    CHECK(!sawTurn);                     // and not turned into
    CHECK(s.pose.x > 700.0);
    CHECK(s.crossTrackError() < 15.0);
  }

  // ------------------------------------------------------------- short gap
  {
    TEST("it coasts across a short gap in the line");

    Sim s;
    Track t = sim::makeGap(400.0, 40.0, 1200.0);
    CHECK(startOnLine(s, t, Pose{0.0, 0.0, 0.0}));

    bool sawRediscover = false;
    for (int i = 0; i < 6000; i++) {
      s.tick();
      if (s.nav.state() == NavState::Rediscover) sawRediscover = true;
      if (s.pose.x > 700.0 || s.nav.state() == NavState::Halted) break;
    }

    printf("    short gap: ended at (%.0f, %.0f), state %s, error %.1f mm, "
           "rediscover seen %d\n",
           s.pose.x, s.pose.y, stateName(s.nav.state()), s.crossTrackError(),
           (int)sawRediscover);

    CHECK(s.nav.state() != NavState::Halted);
    CHECK(s.pose.x > 700.0);
    CHECK(s.crossTrackError() < 15.0);
  }

  // -------------------------------------------------------------- long gap
  {
    TEST("it halts when the line is gone for good");

    Sim s;
    // A line that simply stops: the course has ended, or the gap is longer
    // than the robot can coast across.
    Track t = sim::makeStraight(400.0);
    CHECK(startOnLine(s, t, Pose{0.0, 0.0, 0.0}));

    s.runUntil(8000, [](const Sim &x) { return x.nav.state() == NavState::Halted; });

    printf("    long gap: state %s after %u iterations, ended at (%.0f, %.0f)\n",
           stateName(s.nav.state()), (unsigned)s.iterations, s.pose.x, s.pose.y);

    CHECK(s.nav.state() == NavState::Halted);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 0);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 0);

    // Halted must stay halted with the motors off.
    s.runFor(50);
    CHECK(s.nav.state() == NavState::Halted);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 0);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 0);
  }

  // ------------------------------------------------------------- full lap
  {
    TEST("it completes a lap of a closed course");

    Sim s;
    Track t = sim::makeClosedLap(600.0, 300.0, 150.0);
    CHECK(startOnLine(s, t, Pose{20.0, 0.0, 0.0}));

    // Four quadrant markers, which must be reached in order before the robot
    // can be said to have gone round rather than shuffled about near the start.
    const double markX[4] = { 600.0, 750.0, 300.0, -150.0 };
    const double markY[4] = { 0.0,   300.0, 600.0,  300.0 };
    int reached = 0;
    double worst = 0.0;
    uint32_t lapIterations = 0;

    const uint32_t BUDGET = 40000;
    for (uint32_t i = 0; i < BUDGET; i++) {
      s.tick();
      lapIterations++;

      const double e = s.crossTrackError();
      if (e > worst) worst = e;

      if (reached < 4 &&
          std::hypot(s.pose.x - markX[reached], s.pose.y - markY[reached]) < 60.0) {
        reached++;
      }

      if (reached == 4 && std::hypot(s.pose.x - 20.0, s.pose.y) < 60.0) break;
      if (s.nav.state() == NavState::Halted) break;
    }

    const double seconds = (double)lapIterations * (double)CONTROL_PERIOD_US * 1e-6;

    printf("    lap: %d of 4 markers, %u iterations (%.1f simulated seconds), "
           "worst error %.1f mm, state %s, ended at (%.0f, %.0f)\n",
           reached, (unsigned)lapIterations, seconds, worst,
           stateName(s.nav.state()), s.pose.x, s.pose.y);

    CHECK_EQ(reached, 4);
    CHECK(s.nav.state() != NavState::Halted);
    CHECK(worst < 20.0);
    CHECK(lapIterations < BUDGET);
  }

  return testSummary("simulation");
}
