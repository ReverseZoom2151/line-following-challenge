#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "config.h"
#include "linesensors.h"
#include "motors.h"
#include "pid.h"

// The states the robot can be in. Scoped enum rather than a set of #defines,
// so an unrelated integer cannot be assigned into the state variable by
// mistake and the compiler can check every switch.
enum class NavState : uint8_t {

  Calibrate,   // sweeping across the line to learn light and dark
  JoinLine,    // driving forward from the start box, looking for the line
  FollowLine,  // steering along the line under PID control
  TurnLeft,    // taking a left branch
  TurnRight,   // taking a right branch
  Crossroads,  // driving across a four-way junction
  Rediscover,  // line lost, searching for it again
  Halted       // stopped and staying stopped

};

// Event-driven state machine. Every transition goes through enter(), which
// runs the exit action of the state being left and the entry action of the
// state being joined. Timers and the controller are therefore reset in
// exactly one place, rather than at each of the scattered assignments the
// previous sketch made to a bare int.
class Navigator_c {

  private:

    LineSensor_c *sensors = 0;
    Motors_c *motors = 0;

    PID_c pid = PID_c(PID_KP, PID_KI, PID_KD, -MAX_PWM, MAX_PWM, PID_INTEGRAL_LIMIT);

    NavState nav_state = NavState::Calibrate;
    uint32_t state_entered_ms = 0;
    uint32_t now_ms = 0;

    // JoinLine needs to see the line twice, debounced, before it commits:
    // the start box is crossed at an angle and one brush is not enough.
    uint8_t join_hits = 0;
    uint32_t last_join_hit_ms = 0;
    static const uint32_t JOIN_DEBOUNCE_MS = 200;
    static const uint8_t JOIN_HITS_REQUIRED = 2;

    uint32_t elapsedInState() const { return now_ms - state_entered_ms; }

    // ------------------------------------------------------ entry and exit

    void enter(NavState next) {

      // exit action of the state being left: every state that drives the
      // motors gives them up here, so no state can leave the robot moving
      // under a command the next state did not ask for
      switch (nav_state) {
        case NavState::Calibrate:
        case NavState::TurnLeft:
        case NavState::TurnRight:
        case NavState::Crossroads:
        case NavState::Rediscover:
          motors->stop();
          break;
        default:
          break;
      }

      nav_state = next;
      state_entered_ms = now_ms;

      // entry action of the state being joined
      switch (next) {

        case NavState::Calibrate:
          sensors->beginCalibration();
          break;

        case NavState::JoinLine:
          join_hits = 0;
          last_join_hit_ms = 0;
          break;

        case NavState::FollowLine:
          // the error history from before a turn describes a line the robot
          // is no longer on, so it is dropped rather than carried across
          pid.reset();
          break;

        case NavState::Halted:
          motors->stop();
          break;

        default:
          break;

      }

    }

    // ---------------------------------------------------------- the states

    void runCalibrate(const SensorSnapshot &s) {

      sensors->updateCalibration(s);

      if (elapsedInState() >= CALIBRATION_MS) {
        sensors->endCalibration();
        enter(NavState::JoinLine);
        return;
      }

      // sweeps the sensor bar across the line and the background
      motors->spinRight(BASE_SPEED_PWM, 0);

    }

    void runJoinLine(const SensorSnapshot &s) {

      motors->driveStraight(BASE_SPEED_PWM);

      if (sensors->onLine(s) && (now_ms - last_join_hit_ms) > JOIN_DEBOUNCE_MS) {
        join_hits++;
        last_join_hit_ms = now_ms;
      }

      if (join_hits >= JOIN_HITS_REQUIRED) enter(NavState::FollowLine);

    }

    void runFollowLine(const SensorSnapshot &s) {

      bool line_found = false;
      float position = sensors->linePosition(s, line_found);

      // junctions are decided before steering, so a branch is not steered
      // into as though it were the line drifting
      bool left = sensors->farLeftActive(s);
      bool right = sensors->farRightActive(s);

      if (left && right) {
        enter(NavState::Crossroads);
        return;
      }

      if (left) {
        enter(NavState::TurnLeft);
        return;
      }

      if (right) {
        enter(NavState::TurnRight);
        return;
      }

      if (!line_found) {
        enter(NavState::Rediscover);
        return;
      }

      float steer = pid.update(position, now_ms);

      motors->setMotorPower(BASE_SPEED_PWM + steer, BASE_SPEED_PWM - steer);

    }

    void runTurnLeft(const SensorSnapshot &s) {

      (void)s;
      motors->spinLeft(BASE_SPEED_PWM, 250);
      enter(NavState::FollowLine);

    }

    void runTurnRight(const SensorSnapshot &s) {

      (void)s;
      motors->spinRight(BASE_SPEED_PWM, 250);
      enter(NavState::FollowLine);

    }

    void runCrossroads(const SensorSnapshot &s) {

      // drives across the junction and resumes on the far side
      motors->driveStraight(BASE_SPEED_PWM);

      if (!sensors->farLeftActive(s) && !sensors->farRightActive(s)) {
        enter(NavState::FollowLine);
      }

    }

    void runRediscover(const SensorSnapshot &s) {

      if (sensors->onLine(s) || sensors->farLeftActive(s) || sensors->farRightActive(s)) {
        enter(NavState::FollowLine);
        return;
      }

      if (elapsedInState() >= REDISCOVER_TIMEOUT_MS) {
        // the course has ended, or the line is gone for good
        enter(NavState::Halted);
        return;
      }

      motors->driveStraight(BASE_SPEED_PWM);

    }

    void runHalted() {

      // idempotent: re-asserting the stop costs two register writes and
      // needs no trap loop to stay put
      motors->stop();

    }

  public:

    Navigator_c() {}

    void begin(LineSensor_c *line_sensors, Motors_c *drive) {

      sensors = line_sensors;
      motors = drive;

      nav_state = NavState::Calibrate;
      state_entered_ms = 0;
      now_ms = 0;
      join_hits = 0;
      last_join_hit_ms = 0;

      pid.reset();
      sensors->beginCalibration();

    }

    void update(uint32_t millis_now, const SensorSnapshot &s) {

      if (sensors == 0 || motors == 0) return;

      now_ms = millis_now;

      switch (nav_state) {
        case NavState::Calibrate:  runCalibrate(s);  break;
        case NavState::JoinLine:   runJoinLine(s);   break;
        case NavState::FollowLine: runFollowLine(s); break;
        case NavState::TurnLeft:   runTurnLeft(s);   break;
        case NavState::TurnRight:  runTurnRight(s);  break;
        case NavState::Crossroads: runCrossroads(s); break;
        case NavState::Rediscover: runRediscover(s); break;
        case NavState::Halted:     runHalted();      break;
      }

    }

    NavState state() const { return nav_state; }

};

#endif
