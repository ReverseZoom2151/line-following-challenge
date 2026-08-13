// Minimal Arduino API stub so firmware headers compile and run on a host.
//
// The firmware is written against the Arduino core, which only exists for the
// target MCU. This stub supplies the handful of functions the sketch actually
// uses, backed by a virtual clock and a virtual pin array, so the pure logic
// can be executed and asserted on under g++.
//
// Two behaviours matter for testing this particular firmware:
//
//   1. Virtual time. Nothing here blocks. digitalRead() advances the clock by
//      one microsecond per call, so the firmware's busy-wait discharge loops
//      make progress and terminate instead of spinning forever.
//
//   2. A watchdog. A read loop with no timeout would still never exit, which
//      on hardware means the robot hangs. Rather than hang the test suite,
//      digitalRead() gives up after MOCK_WATCHDOG_READS and sets
//      mockWatchdogTripped. A test can then assert on that flag, which is a
//      direct, executable proof that the firmware would have hung on the robot.
//
// Note: unsigned long is 32-bit on AVR and 64-bit on a typical Linux host, so
// millis()/micros() rollover behaviour is not reproduced here. The virtual
// clock is uint32_t to keep the value range faithful.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>

// ---------------------------------------------------------------- constants

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// ATmega32U4 analog pin numbering, as used by the sketch.
#define A0 18
#define A1 19
#define A2 20
#define A3 21
#define A4 22
#define A5 23

// On AVR, F() moves a string literal into flash to save the 2.5 KB of RAM.
// On a host there is no such distinction, so it degrades to the literal.
#define F(str) (str)

static const int MOCK_MAX_PINS = 32;

// Give up on a discharge loop after this many reads and flag it. Real reads
// finish in a few hundred iterations; anything beyond this is a hang.
static const unsigned long MOCK_WATCHDOG_READS = 100000UL;

// ------------------------------------------------------------- virtual pins

struct MockPin {
  int mode = INPUT;
  int digitalValue = LOW;   // last value written with digitalWrite
  int analogValue = 0;      // last duty written with analogWrite
  int analogWrites = 0;     // how many times analogWrite touched this pin

  // How long this pin reads HIGH after being switched to INPUT, modelling the
  // RC discharge of a reflectance sensor. 0 means it reads LOW immediately.
  unsigned long dischargeMicros = 0;

  // Set true to model a sensor that never discharges: a fully black surface,
  // or a disconnected pin.
  bool neverDischarges = false;

  uint32_t inputSinceMicros = 0;  // when the pin last became INPUT
};

inline MockPin mockPins[MOCK_MAX_PINS];
inline uint32_t mockNowMicros = 0;
inline bool mockWatchdogTripped = false;
inline unsigned long mockDigitalReadCount = 0;

// ------------------------------------------------------------- test control

inline void mockReset() {
  for (int i = 0; i < MOCK_MAX_PINS; i++) mockPins[i] = MockPin();
  mockNowMicros = 0;
  mockWatchdogTripped = false;
  mockDigitalReadCount = 0;
}

inline void mockAdvanceMicros(uint32_t us) { mockNowMicros += us; }

// Model a sensor whose capacitor discharges after `us` microseconds. Lower
// values mean a more reflective (whiter) surface.
inline void mockSetDischarge(int pin, unsigned long us) {
  mockPins[pin].dischargeMicros = us;
  mockPins[pin].neverDischarges = false;
}

// Model a sensor that never releases: black surface or broken wiring.
inline void mockSetNeverDischarges(int pin) {
  mockPins[pin].neverDischarges = true;
}

// ---------------------------------------------------------------- Arduino API

inline void pinMode(int pin, int mode) {
  if (pin < 0 || pin >= MOCK_MAX_PINS) return;
  // Switching to INPUT starts the discharge window.
  if (mode == INPUT && mockPins[pin].mode != INPUT) {
    mockPins[pin].inputSinceMicros = mockNowMicros;
  }
  mockPins[pin].mode = mode;
}

inline void digitalWrite(int pin, int value) {
  if (pin < 0 || pin >= MOCK_MAX_PINS) return;
  mockPins[pin].digitalValue = value;
}

inline void analogWrite(int pin, int value) {
  if (pin < 0 || pin >= MOCK_MAX_PINS) return;
  mockPins[pin].analogValue = value;
  mockPins[pin].analogWrites++;
}

inline int digitalRead(int pin) {
  if (pin < 0 || pin >= MOCK_MAX_PINS) return LOW;

  // Every read costs a microsecond, so busy-wait loops make progress.
  mockNowMicros++;
  mockDigitalReadCount++;

  if (mockDigitalReadCount > MOCK_WATCHDOG_READS) {
    // The firmware is stuck. Break the loop and record that it happened.
    mockWatchdogTripped = true;
    return LOW;
  }

  const MockPin &p = mockPins[pin];
  if (p.mode != INPUT) return p.digitalValue;
  if (p.neverDischarges) return HIGH;

  return (mockNowMicros - p.inputSinceMicros) < p.dischargeMicros ? HIGH : LOW;
}

inline unsigned long micros() { return mockNowMicros; }
inline unsigned long millis() { return mockNowMicros / 1000UL; }

inline void delay(unsigned long ms) { mockNowMicros += ms * 1000UL; }
inline void delayMicroseconds(unsigned int us) { mockNowMicros += us; }

// ---------------------------------------------------------------- Serial

// The firmware prints diagnostics. Tests do not assert on them, so this
// swallows output while keeping a count for the odd sanity check.
struct MockSerial {
  int printCalls = 0;

  void begin(long) {}
  template <typename T> void print(T) { printCalls++; }
  template <typename T> void print(T, int) { printCalls++; }
  template <typename T> void println(T) { printCalls++; }
  template <typename T> void println(T, int) { printCalls++; }
  void println() { printCalls++; }
  operator bool() const { return true; }
};

inline MockSerial Serial;
