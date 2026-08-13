#ifndef PID_H
#define PID_H

#include "config.h"

// A small PID controller with the two safety properties this robot needs:
// the integral term cannot wind up while the output is saturated, and the
// first sample after construction or reset produces no derivative kick.
//
// With Ki and Kd at zero it reduces exactly to the proportional controller
// the robot is known to drive with, which is the shipped default: there is no
// hardware available on which to tune the other two terms.
class PID_c {

  private:

    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;
    float integral_limit;

    float integral;
    float prev_error;
    uint32_t prev_ms;
    bool has_previous;  // false until the first sample has been seen

  public:

    PID_c(float p_gain, float i_gain, float d_gain,
          float output_min, float output_max, float i_limit)
      : kp(p_gain), ki(i_gain), kd(d_gain),
        out_min(output_min), out_max(output_max), integral_limit(i_limit),
        integral(0.0f), prev_error(0.0f), prev_ms(0), has_previous(false) {}

    // Clears the accumulated state. Called on every state entry that starts a
    // fresh line-following run, so error accumulated before a turn does not
    // steer the robot after it.
    void reset() {

      integral = 0.0f;
      prev_error = 0.0f;
      prev_ms = 0;
      has_previous = false;

    }

    void setGains(float p_gain, float i_gain, float d_gain) {

      kp = p_gain;
      ki = i_gain;
      kd = d_gain;

    }

    float update(float error, uint32_t now_ms) {

      float derivative = 0.0f;

      if (!has_previous) {

        // No previous sample, so there is no time base and no rate of change.
        // Reporting one would put a large spurious spike on the very first
        // control iteration, when the error is typically at its largest.
        has_previous = true;

      } else {

        uint32_t dt_ms = now_ms - prev_ms;

        // dt of zero means two updates within the same millisecond: integrate
        // nothing and differentiate nothing rather than divide by zero
        if (dt_ms > 0) {

          float dt = (float)dt_ms * 0.001f;

          integral += error * dt;

          // anti-windup: the accumulator is bounded, so a long saturation
          // cannot store up a correction that overshoots on the way back
          if (integral > integral_limit) integral = integral_limit;
          else if (integral < -integral_limit) integral = -integral_limit;

          derivative = (error - prev_error) / dt;

        }

      }

      prev_error = error;
      prev_ms = now_ms;

      float output = (kp * error) + (ki * integral) + (kd * derivative);

      if (output > out_max) output = out_max;
      else if (output < out_min) output = out_min;

      return output;

    }

};

#endif
