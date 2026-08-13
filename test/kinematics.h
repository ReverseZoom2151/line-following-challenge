// Differential drive kinematics for the host-side course simulator.
//
// This is a MODEL of a two-wheeled robot, not a measurement of one. Every
// number below is a nominal figure taken from the published dimensions of the
// Pololu 3Pi+ 32U4 or, where no published figure exists, an openly stated
// guess. Nothing here has been checked against a real chassis, because no
// robot was available. Treat the constants as assumptions to be revisited the
// first time the firmware runs on hardware.
//
// What the model does capture:
//   - the geometric relationship between two wheel speeds and the resulting
//     forward and angular velocity of the body
//   - exact integration of that velocity over a timestep, so a constant
//     command traces a true arc rather than a chord
//   - a motor deadband, because a real brushed motor does not turn at PWM 5
//
// What it deliberately does not capture, and cannot:
//   - wheel slip, and the loss of heading that comes with it
//   - motor nonlinearity, gearbox backlash, or the difference between the two
//     motors on the same robot
//   - battery voltage sag, which changes the PWM-to-speed map as a run goes on
//   - the mass of the robot: commands take effect instantly here, with no
//     acceleration limit and no momentum
//
// A run that succeeds in this model has demonstrated that the control LOGIC is
// self-consistent. It has not demonstrated that the robot drives.

#pragma once

#include <cmath>

namespace sim {

// M_PI is not part of standard C++ and is not defined under -std=c++17, so the
// simulator carries its own.
constexpr double PI = 3.14159265358979323846;

// ------------------------------------------------------------- geometry

// Nominal figures for the 3Pi+ 32U4. The wheel diameter is the published
// nominal size of the moulded wheel, which is not the same as its effective
// rolling diameter under load. The track width is the approximate distance
// between the two wheel contact patches, measured off the published board
// outline rather than off a robot.
constexpr double WHEEL_DIAMETER_MM = 32.0;
constexpr double TRACK_WIDTH_MM = 96.0;

// ---------------------------------------------------------- motor model

// A guess, and the least defensible constant in this file. It stands for the
// free-running wheel speed at full PWM on a fresh battery with the 30:1
// gearmotors. The real figure depends on the motor option fitted, the battery
// state and the load, and can differ by a factor of two or more between
// builds.
constexpr double MAX_WHEEL_RPM = 300.0;

// Below this duty the motor produces less torque than it needs to break
// stiction and the wheel does not turn at all. Modelled as a hard threshold
// with a linear region above it; the real transition is soft, load dependent
// and different for each motor. This constant matters more than it looks:
// BASE_SPEED_PWM in the firmware is 30 out of 255, which sits only just above
// the deadband, so a modest error here changes the modelled speed a lot.
constexpr double PWM_DEADBAND = 10.0;

constexpr double PWM_MAX = 255.0;

// Peak wheel speed in mm/s, derived from the two constants above so that the
// wheel diameter is actually load bearing rather than decorative.
constexpr double MAX_WHEEL_SPEED_MMPS =
    (MAX_WHEEL_RPM / 60.0) * PI * WHEEL_DIAMETER_MM;

// Maps a signed PWM demand onto a wheel speed in mm/s. Linear above the
// deadband, zero inside it, saturating at full scale. A real motor is none of
// those three things exactly.
inline double wheelSpeed(double pwm) {

  double magnitude = std::fabs(pwm);

  if (magnitude <= PWM_DEADBAND) return 0.0;
  if (magnitude > PWM_MAX) magnitude = PWM_MAX;

  double normalised = (magnitude - PWM_DEADBAND) / (PWM_MAX - PWM_DEADBAND);
  double speed = normalised * MAX_WHEEL_SPEED_MMPS;

  return (pwm < 0.0) ? -speed : speed;

}

// ------------------------------------------------------------- the pose

// Position of the centre of the wheel axle, in millimetres, plus the heading
// in radians. Heading 0 points along +x; positive heading rotates towards +y,
// which is the robot's left. That convention matches the firmware's own sign
// rule, where a line left of centre gives a negative linePosition() and a
// negative steer, which slows the left wheel and swings the robot to the left.
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
};

inline double wrapAngle(double a) {
  while (a > PI) a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}

// Advances the pose by dt seconds under a constant pair of PWM demands.
//
// Both wheel speeds are held constant across the step, so the body follows a
// circular arc of radius v/omega. Integrating that arc exactly rather than
// taking a straight-line Euler step means the result does not depend on the
// timestep for a constant command, which keeps a tight turn honest at a 5 ms
// control period.
inline Pose step(const Pose &in, double leftPwm, double rightPwm, double dt) {

  const double vl = wheelSpeed(leftPwm);
  const double vr = wheelSpeed(rightPwm);

  const double v = 0.5 * (vl + vr);                 // forward, mm/s
  const double omega = (vr - vl) / TRACK_WIDTH_MM;  // rad/s, positive is left

  Pose out = in;

  if (std::fabs(omega) < 1e-9) {

    out.x += v * std::cos(in.heading) * dt;
    out.y += v * std::sin(in.heading) * dt;

  } else {

    const double h1 = in.heading + omega * dt;
    const double radius = v / omega;

    out.x += radius * (std::sin(h1) - std::sin(in.heading));
    out.y -= radius * (std::cos(h1) - std::cos(in.heading));
    out.heading = h1;

  }

  out.heading = wrapAngle(out.heading);

  return out;

}

// Convenience for reporting: how fast the body is moving under a command.
inline double bodySpeed(double leftPwm, double rightPwm) {
  return 0.5 * (wheelSpeed(leftPwm) + wheelSpeed(rightPwm));
}

inline double bodyRate(double leftPwm, double rightPwm) {
  return (wheelSpeed(rightPwm) - wheelSpeed(leftPwm)) / TRACK_WIDTH_MM;
}

}  // namespace sim
