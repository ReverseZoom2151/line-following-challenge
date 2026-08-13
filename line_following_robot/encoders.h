#ifndef ENCODERS_H
#define ENCODERS_H

#include "config.h"

// =========================================================================
// UNRESOLVED HARDWARE CONFLICT: THE RIGHT ENCODER SHARES A PIN WITH DN5
// =========================================================================
//
// Pololu's own 3Pi+ 32U4 library puts the right encoder on pins 23 and 22,
// which in the A-Star pin numbering are A5 and A4.
//
// config.h in this project already uses A4:
//
//     static const uint8_t LS_RIGHT_PIN = A4;  // DN5, far right
//
// Both cannot be true. Either the line sensor map in config.h is wrong, or
// the encoder pin numbers below are wrong, or this board revision wires
// something differently from the library's assumption. It has not been
// possible to determine which, because there is no robot to look at.
//
// NOTHING HERE IS WIRED INTO THE SKETCH UNTIL THAT IS SETTLED. If the two
// were both driven, every reflectance reading would fight the encoder input
// on the same pin: DN5 would report nonsense, the far-right junction
// detection built on it would fire at random, and the robot would throw
// itself into turns that are not there.
//
// To settle it, on the bench: check the board silkscreen and the 3Pi+
// schematic for the actual DN5 and right-encoder pins, then correct whichever
// of the two maps is wrong. The pin numbers below are constants precisely so
// that this is a one-line change.
//
// A SECOND, SEPARATE PROBLEM WITH THE SAME PINS: on the ATmega32U4, A4 and A5
// are PF1 and PF0. Those port F pins have neither an external interrupt nor a
// pin-change interrupt. attachInterrupt() on them cannot work, whatever the
// pin map says. Pololu's library does not use attachInterrupt for this reason.
// So the right-hand channel below is written in the same shape as the left for
// clarity and testability, but it will need either different pins or a
// different interrupt mechanism on real hardware. This is also unverified.
//
// =========================================================================
//
// What this class buys, once the above is resolved: every turn constant in
// this firmware is currently a guess. TURN_TIMEOUT_MS exists because the
// robot has no idea how far it has actually turned, so it spins for a while
// and hopes. headingChangeDegrees() replaces the guess with a measurement.
//
// The constants below are declared here for now so this header stands alone.
// They belong in config.h once this is wired into the sketch.

// ---------------------------------------------------------------- pin map

// Left encoder: pins 7 (PE6, external interrupt INT6) and 8 (PB4, pin-change
// interrupt). Both are free in this project's pin map.
static const uint8_t ENCODER_LEFT_A_PIN  = 7;
static const uint8_t ENCODER_LEFT_B_PIN  = 8;

// Right encoder: A5 and A4. See the conflict notice above before using these.
static const uint8_t ENCODER_RIGHT_A_PIN = A5;
static const uint8_t ENCODER_RIGHT_B_PIN = A4;

// ------------------------------------------------------------- geometry

// Pololu quote about 12 counts per revolution of the motor shaft, counting
// both edges of both channels. THE GEAR RATIO VARIES BY EDITION: the 3Pi+ is
// sold in 75:1 (standard), 30:1 (turtle) and 15:1 (hyper) versions and they
// are not distinguishable in software. Confirm which board this is before
// trusting any distance this class reports.
static const float ENCODER_CPR_MOTOR_SHAFT = 12.0f;
static const float ENCODER_GEAR_RATIO      = 75.0f;

// The handlers below count one edge of channel A only, which is a quarter of
// what the 12 CPR figure describes. Counting all four edges needs an
// interrupt on both channels of both encoders, which the pin situation above
// does not currently allow. This factor keeps the arithmetic honest about
// that: 12 * 0.25 * 75 = 225 counts per turn of the wheel.
static const float ENCODER_EDGE_FRACTION_COUNTED = 0.25f;

static const float ENCODER_COUNTS_PER_WHEEL_REV =
    ENCODER_CPR_MOTOR_SHAFT * ENCODER_EDGE_FRACTION_COUNTED * ENCODER_GEAR_RATIO;

// 3Pi+ wheel diameter and the distance between the two wheel contact
// patches, from Pololu's documentation. Both should be measured with a
// ruler and corrected: the track width in particular feeds straight into
// headingChangeDegrees(), so a five per cent error there is a five per cent
// error on every turn the robot makes.
static const float WHEEL_DIAMETER_MM   = 32.0f;
static const float WHEEL_TRACK_WIDTH_MM = 96.0f;

static const float WHEEL_CIRCUMFERENCE_MM = 3.14159265f * WHEEL_DIAMETER_MM;

static const float ENCODER_DEGREES_PER_RADIAN = 57.2957795f;

// Which way a wheel is turning when its two channels agree is a property of
// how the encoder happens to be soldered and which way round the motor is
// wired, and the right-hand motor is mirrored. These make it a one-character
// fix when the robot drives forward and one count goes negative. Both are
// guesses until someone pushes the robot forward by hand and watches.
static const int8_t ENCODER_LEFT_SIGN  = 1;
static const int8_t ENCODER_RIGHT_SIGN = 1;

// ------------------------------------------------------- interrupt guard

// The real Arduino core supplies noInterrupts()/interrupts() as macros. The
// host test stub does not, so they are supplied here for the host only. The
// #ifndef means the core's versions always win when compiling for the target.
#if !defined(ARDUINO) && !defined(__AVR__)
  #ifndef noInterrupts
    // Host stand-ins. They count entries as well as nesting so that a test
    // can prove the guard was actually taken around a counter read, which is
    // otherwise invisible on a machine where nothing fires interrupts.
    inline int mockInterruptGuardDepth = 0;
    inline int mockInterruptGuardEntries = 0;
    inline void noInterrupts() { mockInterruptGuardDepth++; mockInterruptGuardEntries++; }
    inline void interrupts()   { mockInterruptGuardDepth--; }
  #endif
#endif

// ------------------------------------------------------------- counters

// The counters live at namespace scope rather than as class members because
// the interrupt handlers are static: an ISR is called by the hardware with no
// object to work on, so there is nothing for it to reach a member through.
//
// `static` here gives internal linkage, one copy per translation unit. That
// is safe only because this firmware is a single translation unit: the sketch
// includes every header and nothing is compiled separately. If any of this is
// ever moved into a .cpp, these must become a single definition with extern
// declarations, or the handlers will count into one copy while the reader
// reads a different one and every distance will read zero.
namespace encoder_counts {

  // volatile because an ISR writes them behind the main loop's back. Without
  // it the compiler is entitled to cache a count in a register across a loop
  // and never notice it changing.
  static volatile long left  = 0;
  static volatile long right = 0;

}

class Encoders_c {

  private:

    // A long is four bytes and the 32U4 is an eight-bit machine, so reading
    // one is four separate instructions. An interrupt landing between them
    // returns a value that is half old and half new: pass 255 to 256 and a
    // torn read can report 511 or 0. Neither is a number the wheel ever had.
    //
    // Disabling interrupts for the four instructions makes the read atomic.
    // This is correct because these accessors are only ever called from the
    // main loop, where interrupts are known to be enabled to begin with; a
    // bare interrupts() at the end would be wrong inside an ISR, and the
    // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) pattern from <util/atomic.h> should
    // be used instead if that ever changes.
    static long readAtomic(const volatile long &counter) {

      noInterrupts();
      long value = counter;
      interrupts();
      return value;

    }

  public:

    Encoders_c() {}

    void begin() {

      // Pulled up because the encoder outputs are open on some wirings and a
      // floating input generates counts out of nothing.
      pinMode(ENCODER_LEFT_A_PIN,  INPUT_PULLUP);
      pinMode(ENCODER_LEFT_B_PIN,  INPUT_PULLUP);
      pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
      pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);

      reset();

      // CHANGE rather than RISING: both edges of channel A is twice the
      // resolution for no extra cost. See the header notice about the right
      // channel, where this call cannot work on real hardware as written.
      attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN),  handleLeftA,  CHANGE);
      attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), handleRightA, CHANGE);

    }

    // ------------------------------------------------------------ ISRs

    // Quadrature: on an edge of channel A, channel B tells us which way. The
    // two channels are a quarter cycle apart, so B has one value if A led and
    // the other if B led. Nothing on a host fires these, so the tests call
    // them directly.
    //
    // Kept deliberately short. Everything an ISR does is time stolen from the
    // sensor discharge loop in linesensors.h, which is timing critical, so
    // there is no arithmetic and no Serial output here.
    static void handleLeftA() {

      int a = digitalRead(ENCODER_LEFT_A_PIN);
      int b = digitalRead(ENCODER_LEFT_B_PIN);

      encoder_counts::left += (a == b) ? ENCODER_LEFT_SIGN : -ENCODER_LEFT_SIGN;

    }

    static void handleRightA() {

      int a = digitalRead(ENCODER_RIGHT_A_PIN);
      int b = digitalRead(ENCODER_RIGHT_B_PIN);

      encoder_counts::right += (a == b) ? ENCODER_RIGHT_SIGN : -ENCODER_RIGHT_SIGN;

    }

    // ---------------------------------------------------------- readers

    long leftCount()  const { return readAtomic(encoder_counts::left); }
    long rightCount() const { return readAtomic(encoder_counts::right); }

    // Zeroes both counts together, under one guard, so the two cannot be
    // separated by an interrupt. Call this at the start of a turn, then read
    // headingChangeDegrees() to decide when the turn is finished.
    void reset() {

      noInterrupts();
      encoder_counts::left  = 0;
      encoder_counts::right = 0;
      interrupts();

    }

    // --------------------------------------------------------- geometry

    float leftDistanceMm() const {

      return countsToMm(leftCount());

    }

    float rightDistanceMm() const {

      return countsToMm(rightCount());

    }

    // Heading change since the last reset, from the difference between the
    // two wheel distances over the track width. Positive means anticlockwise,
    // that is, a left turn: the right wheel has travelled further than the
    // left.
    //
    // This assumes the wheels roll without slipping. A spin on the spot on a
    // smooth course scrubs the tyres, so expect the measured angle to
    // overstate the real one. That is still an enormous improvement on
    // TURN_TIMEOUT_MS, which measures nothing at all.
    float headingChangeDegrees() const {

      float difference = rightDistanceMm() - leftDistanceMm();

      // Guard rather than an assumption: a track width of zero would only
      // arise from a mistyped constant, but it would produce an infinity that
      // then propagates into the turn logic as a turn that never completes.
      if (WHEEL_TRACK_WIDTH_MM <= 0.0f) return 0.0f;

      return (difference / WHEEL_TRACK_WIDTH_MM) * ENCODER_DEGREES_PER_RADIAN;

    }

  private:

    static float countsToMm(long counts) {

      if (ENCODER_COUNTS_PER_WHEEL_REV <= 0.0f) return 0.0f;  // divide-by-zero guard

      return ((float)counts / ENCODER_COUNTS_PER_WHEEL_REV) * WHEEL_CIRCUMFERENCE_MM;

    }

};

#endif
