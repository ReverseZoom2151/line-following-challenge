#ifndef BUTTONS_H
#define BUTTONS_H

// The three user buttons on the 3Pi+ 32U4, debounced in time rather than by
// blocking.
//
// Why this exists: the diagnostic bench firmware is selected by the BENCH_MODE
// flag in config.h, so changing mode means editing a file, recompiling and
// reflashing, and a board shipped with the flag left at 1 sits on its stand
// and never follows a line. A button read at run time removes both problems.
//
// Nothing in this file calls delay(). The rest of the firmware keeps its
// control path strictly non-blocking, and a debounce implemented as a sleep
// would blind the line sensors for the duration. Instead update() is handed
// the current millis() value each iteration and decides from elapsed time
// whether a level has been stable long enough to be believed.
//
// No hardware has run any of this. The pin numbers, the debounce interval and
// the assumption that the buttons are wired active low are all taken from the
// documented behaviour of Pololu's library, not from measurement.

#include <stdint.h>

// ---------------------------------------------------------------- pin map

// Pin numbers used by Pololu's 3Pi+ 32U4 library for the three user buttons.
// Confirm these against the revision of the board actually in hand before
// trusting them: Pololu has shipped more than one layout under this name, and
// a wrong pin here reads a floating input as a stuck press.
static const uint8_t BUTTON_A_PIN = 14;
static const uint8_t BUTTON_B_PIN = 30;
static const uint8_t BUTTON_C_PIN = 17;

// The buttons are wired to ground through the switch and held up by the
// internal pull-up, so a pressed button reads LOW.
static const uint8_t BUTTON_PRESSED_LEVEL = LOW;

// ------------------------------------------------------------- identifiers

// Indices into the internal arrays, so BUTTON_A can be used directly as a
// subscript. BUTTON_NONE is deliberately outside the valid range and is what
// every accessor returns when asked about a button that does not exist.
static const uint8_t BUTTON_A     = 0;
static const uint8_t BUTTON_B     = 1;
static const uint8_t BUTTON_C     = 2;
static const uint8_t BUTTON_COUNT = 3;
static const uint8_t BUTTON_NONE  = 255;

// ------------------------------------------------------------- debouncing

// A contact bounces for a few milliseconds after it closes. A level must hold
// steady for at least this long before it is accepted as the real state.
// Twenty milliseconds is the usual figure for a tactile switch and is short
// enough that a press still feels immediate to the operator.
static const uint32_t BUTTON_DEBOUNCE_MS = 20;

class Buttons_c {

  private:

    uint8_t bt_pins[BUTTON_COUNT] = { BUTTON_A_PIN, BUTTON_B_PIN, BUTTON_C_PIN };

    // Three separate notions of state, and they are not interchangeable:
    //
    //   raw_level    what the pin said on the most recent update
    //   stable_level what that level settled to once it stopped changing
    //   press_latch  a stable press has happened and nobody has read it yet
    //
    // Splitting them is what lets a press be reported exactly once. The latch
    // is set on the low-going edge of the stable level and cleared only by
    // wasPressed(), so a press that occurs between two calls to wasPressed()
    // is still waiting there when the caller next asks.
    bool     raw_level[BUTTON_COUNT];
    bool     stable_level[BUTTON_COUNT];
    bool     press_latch[BUTTON_COUNT];
    uint32_t last_change_ms[BUTTON_COUNT];

  public:

    Buttons_c() {

      for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
        raw_level[i]      = false;
        stable_level[i]   = false;
        press_latch[i]    = false;
        last_change_ms[i] = 0;
      }

    }

    void begin() {

      for (uint8_t i = 0; i < BUTTON_COUNT; i++) {

        pinMode(bt_pins[i], INPUT_PULLUP);

        // Start from whatever the pin currently says rather than from a
        // guess. Assuming "released" here would manufacture a press on the
        // first update if the operator is already holding a button down at
        // power-up, which is exactly how a mode select tends to be used.
        bool now_pressed = (digitalRead(bt_pins[i]) == BUTTON_PRESSED_LEVEL);

        raw_level[i]      = now_pressed;
        stable_level[i]   = now_pressed;
        press_latch[i]    = false;
        last_change_ms[i] = 0;

      }

    }

    // Call once per control iteration with the loop's millis() value. Sampling
    // is what drives the debounce: a level that has held for
    // BUTTON_DEBOUNCE_MS since it last changed is promoted to the stable
    // level, and a promotion from released to pressed latches an event.
    void update(uint32_t nowMillis) {

      for (uint8_t i = 0; i < BUTTON_COUNT; i++) {

        bool now_pressed = (digitalRead(bt_pins[i]) == BUTTON_PRESSED_LEVEL);

        if (now_pressed != raw_level[i]) {

          // still bouncing, restart the settling window
          raw_level[i]      = now_pressed;
          last_change_ms[i] = nowMillis;
          continue;

        }

        if (now_pressed == stable_level[i]) continue;  // nothing new to accept

        // Unsigned subtraction, so this stays correct across the millis()
        // rollover at 49.7 days rather than freezing the button.
        if ((uint32_t)(nowMillis - last_change_ms[i]) < BUTTON_DEBOUNCE_MS) continue;

        stable_level[i] = now_pressed;

        if (now_pressed) press_latch[i] = true;  // rising edge of a real press

      }

    }

    // The debounced level, right now. True while the button is held down.
    bool isPressed(uint8_t which) const {

      if (which >= BUTTON_COUNT) return false;

      return stable_level[which];

    }

    // Edge triggered. True once per press, and the latch is cleared as it is
    // read, so a caller polling this in the main loop sees each press exactly
    // once no matter how long the button is held or how slowly it polls.
    bool wasPressed(uint8_t which) {

      if (which >= BUTTON_COUNT) return false;

      if (!press_latch[which]) return false;

      press_latch[which] = false;

      return true;

    }

    // Polls all three and reports the first outstanding press, or BUTTON_NONE.
    //
    // The name describes how it is used, not how it works: it does not wait.
    // The caller loops on it, which keeps the sensors and the watchdog alive
    // while the robot sits waiting for an operator to choose a mode.
    uint8_t waitForAnyPress(uint32_t nowMillis) {

      update(nowMillis);

      for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
        if (wasPressed(i)) return i;
      }

      return BUTTON_NONE;

    }

    // Drops any presses recorded so far. Worth calling when entering a mode
    // that will poll for a button, so the press that selected the mode is not
    // immediately read back as the press that ends it.
    void clearPresses() {

      for (uint8_t i = 0; i < BUTTON_COUNT; i++) press_latch[i] = false;

    }

};

#endif
