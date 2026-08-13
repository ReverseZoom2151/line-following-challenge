// Course geometry and a reflectance sensor model for the host simulator.
//
// A track here is a set of centrelines: straight segments and circular arcs,
// each painted to a fixed line width. A point on the course is black if it
// lies within half a line width of any centreline, white otherwise. A gap in
// the line is simply an absence of centreline, and a crossroads is two
// centrelines that intersect. That is all the geometry there is.
//
// The sensor model then answers one question: given a Pose, what discharge
// time would each of the five reflectance sensors report? The answer is fed
// into the virtual pins of arduino_stub.h so that the REAL firmware reading
// code runs, including its calibration and normalisation, rather than the
// simulator handing the navigator a snapshot it made up.
//
// Honest account of what is modelled:
//
//   - The line is a hard-edged geometric stripe. Real printed or taped lines
//     have fuzzy edges, varying width and varying darkness.
//   - Each sensor sees a circular spot of finite size, and its reading ramps
//     linearly between white and black as the stripe crosses that spot. The
//     ramp is a convenience, not physics: a real sensor's response to partial
//     coverage is neither linear nor symmetric.
//   - Discharge time is taken as a linear function of coverage. In reality the
//     RC discharge is exponential in the photocurrent, so the white-to-black
//     mapping is strongly nonlinear.
//   - There is no noise, no ambient light, no ride-height variation and no
//     difference between the five sensors. A real bar reads five different
//     numbers over the same surface, which is the entire reason the firmware
//     calibrates per sensor.
//
// A control loop that works against this model has been shown to be
// geometrically sane. It has not been shown to cope with a real sensor.

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "arduino_stub.h"
#include "config.h"
#include "kinematics.h"

namespace sim {

// ------------------------------------------------------- course constants

// Typical width of the printed line on these courses. Not measured.
constexpr double LINE_WIDTH_MM = 19.0;

// The five sensors sit in a row across the front of the board, roughly 9 mm
// apart, and forward of the wheel axle. The spacing is close to the published
// figure for the 3Pi+ bar; the forward offset is estimated from the board
// outline and is the number most likely to be wrong. It matters: a longer
// offset makes the robot steer earlier and damps the loop, a shorter one makes
// it steer late and destabilises it.
constexpr double SENSOR_SPACING_MM = 9.0;
constexpr double SENSOR_FORWARD_OFFSET_MM = 40.0;

// Radius of the patch of floor one sensor responds to. A guess, used only to
// give a smooth response as the stripe crosses a sensor instead of a step.
constexpr double SENSOR_SPOT_RADIUS_MM = 4.0;

// Discharge times in microseconds. White discharges quickly, black slowly.
// BLACK_DISCHARGE_US sits just under SENSOR_TIMEOUT_US (2500) so that a
// sensor squarely over the line reports a real reading rather than a timeout.
// On real hardware a matte black line frequently does exceed the timeout, in
// which case the firmware records SENSOR_TIMEOUT_US and sets timedOut, which
// is a path this default does not exercise. DEEP_BLACK_DISCHARGE_US is
// provided so a test can choose to exercise it.
constexpr unsigned long WHITE_DISCHARGE_US = 180;
constexpr unsigned long BLACK_DISCHARGE_US = 2400;
constexpr unsigned long DEEP_BLACK_DISCHARGE_US = 4000;

// ------------------------------------------------------------- segments

enum class SegType { Straight, Arc };

struct Segment {

  SegType type = SegType::Straight;

  // Straight: from (x0, y0) to (x1, y1).
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;

  // Arc: centred on (cx, cy), radius r, swept from angle a0 to a1 in radians.
  // a1 must be greater than a0; the sweep is anticlockwise, but since only the
  // set of painted points matters, a clockwise arc is described by naming its
  // endpoints in the other order.
  double cx = 0.0, cy = 0.0, r = 0.0, a0 = 0.0, a1 = 0.0;

};

inline Segment straight(double x0, double y0, double x1, double y1) {
  Segment s;
  s.type = SegType::Straight;
  s.x0 = x0; s.y0 = y0; s.x1 = x1; s.y1 = y1;
  return s;
}

inline Segment arc(double cx, double cy, double r, double a0Deg, double a1Deg) {
  Segment s;
  s.type = SegType::Arc;
  s.cx = cx; s.cy = cy; s.r = r;
  s.a0 = a0Deg * sim::PI / 180.0;
  s.a1 = a1Deg * sim::PI / 180.0;
  return s;
}

inline double distanceToSegment(const Segment &s, double px, double py) {

  if (s.type == SegType::Straight) {

    const double dx = s.x1 - s.x0;
    const double dy = s.y1 - s.y0;
    const double lenSq = dx * dx + dy * dy;

    double t = 0.0;
    if (lenSq > 1e-12) {
      t = ((px - s.x0) * dx + (py - s.y0) * dy) / lenSq;
      if (t < 0.0) t = 0.0;
      if (t > 1.0) t = 1.0;
    }

    const double qx = s.x0 + t * dx;
    const double qy = s.y0 + t * dy;

    return std::hypot(px - qx, py - qy);

  }

  // Arc. If the point's bearing from the centre falls inside the swept range
  // the nearest point is radial; otherwise it is one of the two endpoints.
  const double vx = px - s.cx;
  const double vy = py - s.cy;
  const double dist = std::hypot(vx, vy);

  double bearing = std::atan2(vy, vx);
  double rel = bearing - s.a0;
  while (rel < 0.0) rel += 2.0 * sim::PI;
  while (rel >= 2.0 * sim::PI) rel -= 2.0 * sim::PI;

  if (rel <= (s.a1 - s.a0)) return std::fabs(dist - s.r);

  const double e0x = s.cx + s.r * std::cos(s.a0);
  const double e0y = s.cy + s.r * std::sin(s.a0);
  const double e1x = s.cx + s.r * std::cos(s.a1);
  const double e1y = s.cy + s.r * std::sin(s.a1);

  const double d0 = std::hypot(px - e0x, py - e0y);
  const double d1 = std::hypot(px - e1x, py - e1y);

  return (d0 < d1) ? d0 : d1;

}

// ---------------------------------------------------------------- track

struct Track {

  std::vector<Segment> segments;
  double lineWidth = LINE_WIDTH_MM;

  // Distance from a point to the nearest painted centreline. Returns a large
  // number for an empty track so that "nowhere near the line" is still a
  // meaningful answer.
  double distanceToCentreline(double px, double py) const {

    double best = 1e9;

    for (const Segment &s : segments) {
      const double d = distanceToSegment(s, px, py);
      if (d < best) best = d;
    }

    return best;

  }

  bool isBlack(double px, double py) const {
    return distanceToCentreline(px, py) <= (lineWidth * 0.5);
  }

  // Fraction of a sensor's spot that is over the line, ramped linearly across
  // the spot diameter. Documented above as a convenience, not physics.
  double coverage(double px, double py) const {

    const double d = distanceToCentreline(px, py);
    const double inner = (lineWidth * 0.5) - SENSOR_SPOT_RADIUS_MM;
    const double outer = (lineWidth * 0.5) + SENSOR_SPOT_RADIUS_MM;

    if (d <= inner) return 1.0;
    if (d >= outer) return 0.0;

    return (outer - d) / (outer - inner);

  }

};

// -------------------------------------------------------- sensor placement

// World position of sensor i, where 0 is the far left sensor (DN1) and 4 the
// far right (DN5). Left is +y, matching the heading convention in
// kinematics.h and the firmware's own left-negative weighting.
//
// The forward offset is a parameter rather than a fixed constant so that a
// test can ask how much of the loop's stability rests on it. It is a guessed
// number, and a result that only holds at one value of a guessed number is
// worth knowing about.
inline void sensorPosition(const Pose &p, int i, double &sx, double &sy,
                           double forwardOffset = SENSOR_FORWARD_OFFSET_MM) {

  const double lateral = ((NUM_SENSORS - 1) * 0.5 - (double)i) * SENSOR_SPACING_MM;

  const double c = std::cos(p.heading);
  const double s = std::sin(p.heading);

  sx = p.x + forwardOffset * c - lateral * s;
  sy = p.y + forwardOffset * s + lateral * c;

}

struct SensorReadings {
  unsigned long us[NUM_SENSORS] = {};
  double coverage[NUM_SENSORS] = {};
};

// What the five sensors would read with the robot at this pose.
inline SensorReadings readTrack(const Track &t, const Pose &p,
                                unsigned long blackUs = BLACK_DISCHARGE_US,
                                double forwardOffset = SENSOR_FORWARD_OFFSET_MM) {

  SensorReadings r;

  for (int i = 0; i < NUM_SENSORS; i++) {

    double sx = 0.0, sy = 0.0;
    sensorPosition(p, i, sx, sy, forwardOffset);

    const double cov = t.coverage(sx, sy);
    const double span = (double)blackUs - (double)WHITE_DISCHARGE_US;

    r.coverage[i] = cov;
    r.us[i] = (unsigned long)((double)WHITE_DISCHARGE_US + cov * span + 0.5);

  }

  return r;

}

// Pushes a set of readings into the virtual pins, so that the firmware's own
// readAll() produces the snapshot rather than the simulator fabricating one.
inline void applyToMockPins(const SensorReadings &r) {

  static const uint8_t pins[NUM_SENSORS] = {
    LS_LEFT_PIN, LS_MIDLEFT_PIN, LS_MIDDLE_PIN, LS_MIDRIGHT_PIN, LS_RIGHT_PIN
  };

  for (int i = 0; i < NUM_SENSORS; i++) {
    mockSetDischarge(pins[i], r.us[i]);
  }

}

// --------------------------------------------------------- course library

// Every open course below starts its line this far behind the origin, which is
// where the robot is placed.
//
// That is not decoration. The firmware calibrates by spinning on the spot, so
// a sensor only learns what black looks like if it passes over the line during
// the sweep. The sweep turns one way only, so a course whose line begins
// exactly at the robot leaves the outer sensors on one side over bare floor
// for the whole sweep, and endCalibration() rejects the result. Real courses
// put the start box on the line with line either side, and these tracks model
// that. The consequences of the other case are asserted on separately, since
// they turn out to be severe.
constexpr double START_LEAD_IN_MM = 200.0;

// A plain straight running along +x through the origin.
inline Track makeStraight(double length = 2000.0,
                          double leadIn = START_LEAD_IN_MM) {
  Track t;
  t.segments.push_back(straight(-leadIn, 0.0, length, 0.0));
  return t;
}

// A right-angle left-hand corner: in along +x to the vertex, then away along
// +y. The two legs meet exactly, with no rounding, which is the hardest case
// for a sensor bar of finite width.
inline Track makeCorner90(double approach = 500.0, double exitLen = 500.0,
                          double leadIn = START_LEAD_IN_MM) {
  Track t;
  t.segments.push_back(straight(-leadIn, 0.0, approach, 0.0));
  t.segments.push_back(straight(approach, 0.0, approach, exitLen));
  return t;
}

// A gentle left-hand curve: a straight run-in, then a quarter circle of the
// given radius. 300 mm is comfortably inside what the chassis can follow.
inline Track makeGentleCurve(double runIn = 300.0, double radius = 300.0,
                             double leadIn = START_LEAD_IN_MM) {
  Track t;
  t.segments.push_back(straight(-leadIn, 0.0, runIn, 0.0));
  t.segments.push_back(arc(runIn, radius, radius, -90.0, 0.0));
  return t;
}

// A four-way junction: the line the robot is following, crossed square by a
// second line at the given distance along it.
inline Track makeCrossroads(double at = 400.0, double length = 900.0,
                            double armLength = 200.0,
                            double leadIn = START_LEAD_IN_MM) {
  Track t;
  t.segments.push_back(straight(-leadIn, 0.0, length, 0.0));
  t.segments.push_back(straight(at, -armLength, at, armLength));
  return t;
}

// A straight with a piece missing. The robot should coast across a short gap
// and give up on a long one, and which of those happens depends on how far it
// travels during REDISCOVER_TIMEOUT_MS.
inline Track makeGap(double gapStart = 400.0, double gapLength = 40.0,
                     double totalLength = 1200.0,
                     double leadIn = START_LEAD_IN_MM) {
  Track t;
  t.segments.push_back(straight(-leadIn, 0.0, gapStart, 0.0));
  t.segments.push_back(straight(gapStart + gapLength, 0.0, totalLength, 0.0));
  return t;
}

// No line anywhere: bare floor.
inline Track makeBlank() {
  return Track();
}

// A closed lap: a rectangle with quarter-circle corners, traversed
// anticlockwise from the origin. No junctions, no gaps, so a full circuit
// exercises nothing but FollowLine.
inline Track makeClosedLap(double width = 600.0, double height = 300.0,
                           double radius = 150.0) {

  Track t;

  const double x0 = 0.0;
  const double x1 = width;
  const double y0 = 0.0;
  const double y1 = height + 2.0 * radius;

  t.segments.push_back(straight(x0, y0, x1, y0));
  t.segments.push_back(arc(x1, y0 + radius, radius, -90.0, 0.0));
  t.segments.push_back(straight(x1 + radius, y0 + radius, x1 + radius, y1 - radius));
  t.segments.push_back(arc(x1, y1 - radius, radius, 0.0, 90.0));
  t.segments.push_back(straight(x1, y1, x0, y1));
  t.segments.push_back(arc(x0, y1 - radius, radius, 90.0, 180.0));
  t.segments.push_back(straight(x0 - radius, y1 - radius, x0 - radius, y0 + radius));
  t.segments.push_back(arc(x0, y0 + radius, radius, 180.0, 270.0));

  return t;

}

}  // namespace sim
