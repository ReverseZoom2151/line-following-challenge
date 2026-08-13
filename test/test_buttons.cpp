// Host tests for buttons.h.
//
// The two things worth proving here are that the debounce is done in time
// rather than by blocking, and that a press is reported exactly once. Both are
// checked by driving the virtual pins and handing update() explicit
// millisecond values, so a whole bounce sequence is exercised without any real
// time passing and without the test depending on delay().

#include "arduino_stub.h"
#include "test_harness.h"

#include "buttons.h"

// mockReset() leaves every pin at LOW, and a 3Pi+ button reads LOW when
// pressed, so a freshly reset board looks like all three buttons are held
// down. Every test starts from an explicitly released state instead.
static void releaseAll() {
  mockReleaseButton(BUTTON_A_PIN);
  mockReleaseButton(BUTTON_B_PIN);
  mockReleaseButton(BUTTON_C_PIN);
}

int main() {
  printf("buttons\n");

  {
    TEST("begin configures all three buttons as pulled-up inputs");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();

    CHECK_EQ(mockPins[BUTTON_A_PIN].mode, INPUT_PULLUP);
    CHECK_EQ(mockPins[BUTTON_B_PIN].mode, INPUT_PULLUP);
    CHECK_EQ(mockPins[BUTTON_C_PIN].mode, INPUT_PULLUP);
  }

  {
    TEST("nothing is pressed on a quiet board");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();

    for (uint32_t t = 0; t < 200; t += 10) buttons.update(t);

    CHECK(!buttons.isPressed(BUTTON_A));
    CHECK(!buttons.isPressed(BUTTON_B));
    CHECK(!buttons.isPressed(BUTTON_C));
    CHECK(!buttons.wasPressed(BUTTON_A));
    CHECK_EQ(buttons.waitForAnyPress(200), BUTTON_NONE);
  }

  {
    // The whole point of the debounce: a level that has not held for the
    // interval is not believed yet, and no time is spent waiting for it.
    TEST("a press is not accepted until it has been stable for the interval");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    mockPressButton(BUTTON_A_PIN);

    buttons.update(100);  // first sighting, settling window opens here
    CHECK(!buttons.isPressed(BUTTON_A));

    buttons.update(100 + BUTTON_DEBOUNCE_MS - 1);
    CHECK(!buttons.isPressed(BUTTON_A));

    buttons.update(100 + BUTTON_DEBOUNCE_MS);
    CHECK(buttons.isPressed(BUTTON_A));
  }

  {
    // A bouncing contact chatters for a few milliseconds. Every transition
    // restarts the window, so the chatter produces no events at all.
    TEST("contact bounce restarts the window and yields one press");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    uint32_t t = 10;
    for (int i = 0; i < 6; i++) {
      if (i % 2 == 0) mockPressButton(BUTTON_A_PIN);
      else            mockReleaseButton(BUTTON_A_PIN);
      buttons.update(t);
      t += 3;  // each edge well inside the debounce interval
      CHECK(!buttons.isPressed(BUTTON_A));
    }

    mockPressButton(BUTTON_A_PIN);
    buttons.update(t);
    buttons.update(t + BUTTON_DEBOUNCE_MS);

    CHECK(buttons.isPressed(BUTTON_A));
    CHECK(buttons.wasPressed(BUTTON_A));
    CHECK(!buttons.wasPressed(BUTTON_A));  // and only one event from all of it
  }

  {
    TEST("wasPressed reports a press once and clears on read");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    mockPressButton(BUTTON_A_PIN);
    buttons.update(10);
    buttons.update(10 + BUTTON_DEBOUNCE_MS);

    CHECK(buttons.wasPressed(BUTTON_A));
    CHECK(!buttons.wasPressed(BUTTON_A));
    CHECK(!buttons.wasPressed(BUTTON_A));

    // holding the button down does not manufacture further events
    for (uint32_t t = 100; t < 500; t += 10) buttons.update(t);
    CHECK(buttons.isPressed(BUTTON_A));
    CHECK(!buttons.wasPressed(BUTTON_A));
  }

  {
    // The latch is what stops a press being lost by a caller that polls
    // wasPressed() less often than update(). The press here is over and done
    // with long before anybody asks about it.
    TEST("a press that ends before it is read is still reported");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    mockPressButton(BUTTON_A_PIN);
    buttons.update(10);
    buttons.update(40);   // press accepted here

    mockReleaseButton(BUTTON_A_PIN);
    buttons.update(50);
    buttons.update(200);  // release accepted here

    CHECK(!buttons.isPressed(BUTTON_A));  // no longer held
    CHECK(buttons.wasPressed(BUTTON_A));  // but the press is not lost
    CHECK(!buttons.wasPressed(BUTTON_A));
  }

  {
    TEST("press, release and press again give two separate events");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    mockPressButton(BUTTON_A_PIN);
    buttons.update(10);
    buttons.update(40);
    CHECK(buttons.wasPressed(BUTTON_A));

    mockReleaseButton(BUTTON_A_PIN);
    buttons.update(50);
    buttons.update(80);
    CHECK(!buttons.isPressed(BUTTON_A));

    mockPressButton(BUTTON_A_PIN);
    buttons.update(90);
    buttons.update(120);
    CHECK(buttons.wasPressed(BUTTON_A));
    CHECK(!buttons.wasPressed(BUTTON_A));
  }

  {
    TEST("a release is debounced the same way a press is");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    mockPressButton(BUTTON_A_PIN);
    buttons.update(10);
    buttons.update(40);
    CHECK(buttons.isPressed(BUTTON_A));

    mockReleaseButton(BUTTON_A_PIN);
    buttons.update(100);
    CHECK(buttons.isPressed(BUTTON_A));  // release not believed yet

    buttons.update(100 + BUTTON_DEBOUNCE_MS - 1);
    CHECK(buttons.isPressed(BUTTON_A));

    buttons.update(100 + BUTTON_DEBOUNCE_MS);
    CHECK(!buttons.isPressed(BUTTON_A));
  }

  {
    TEST("the three buttons are independent");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    mockPressButton(BUTTON_B_PIN);
    buttons.update(10);
    buttons.update(40);

    CHECK(buttons.isPressed(BUTTON_B));
    CHECK(!buttons.isPressed(BUTTON_A));
    CHECK(!buttons.isPressed(BUTTON_C));
    CHECK(buttons.wasPressed(BUTTON_B));
    CHECK(!buttons.wasPressed(BUTTON_A));
    CHECK(!buttons.wasPressed(BUTTON_C));
  }

  {
    TEST("waitForAnyPress reports which button, once, without blocking");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();

    CHECK_EQ(buttons.waitForAnyPress(0), BUTTON_NONE);

    mockPressButton(BUTTON_C_PIN);

    CHECK_EQ(buttons.waitForAnyPress(10), BUTTON_NONE);  // still settling
    CHECK_EQ(buttons.waitForAnyPress(40), BUTTON_C);
    CHECK_EQ(buttons.waitForAnyPress(50), BUTTON_NONE);  // event consumed
  }

  {
    // If the operator is already holding a button when the robot powers up,
    // that held level is the starting state and must not read as a fresh
    // press: a mode select would otherwise fire itself on boot.
    TEST("a button held through begin does not latch a press");
    mockReset();
    releaseAll();
    mockPressButton(BUTTON_A_PIN);

    Buttons_c buttons;
    buttons.begin();

    for (uint32_t t = 0; t < 200; t += 10) buttons.update(t);

    CHECK(buttons.isPressed(BUTTON_A));   // it is genuinely held
    CHECK(!buttons.wasPressed(BUTTON_A));  // but no edge happened
  }

  {
    TEST("clearPresses drops an outstanding event");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    mockPressButton(BUTTON_A_PIN);
    buttons.update(10);
    buttons.update(40);

    buttons.clearPresses();
    CHECK(!buttons.wasPressed(BUTTON_A));
    CHECK(buttons.isPressed(BUTTON_A));  // the level is untouched
  }

  {
    // millis() wraps to zero after about 49.7 days. Unsigned subtraction keeps
    // the elapsed-time comparison correct across the wrap; a signed or naive
    // comparison would leave the button dead until the next reboot.
    TEST("debouncing survives the millis rollover");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0xFFFFFF00UL);

    mockPressButton(BUTTON_A_PIN);
    buttons.update(0xFFFFFFF0UL);
    CHECK(!buttons.isPressed(BUTTON_A));

    buttons.update(0x00000004UL);  // 20 ms later, across the wrap
    CHECK(buttons.isPressed(BUTTON_A));
    CHECK(buttons.wasPressed(BUTTON_A));
  }

  {
    TEST("an out of range index is refused rather than read off the end");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();
    buttons.update(0);

    CHECK(!buttons.isPressed(BUTTON_NONE));
    CHECK(!buttons.wasPressed(BUTTON_NONE));
    CHECK(!buttons.isPressed(BUTTON_COUNT));
    CHECK(!buttons.wasPressed(99));
  }

  {
    // Not an assertion about correctness so much as about style: the control
    // path must not stall. If update() ever grew a delay() the virtual clock
    // would jump, because the stub implements delay() by advancing it.
    TEST("update spends no time on the virtual clock");
    mockReset();
    releaseAll();
    Buttons_c buttons;
    buttons.begin();

    mockPressButton(BUTTON_A_PIN);
    uint32_t before = micros();
    for (uint32_t t = 0; t < 500; t += 10) buttons.update(t);
    uint32_t spent = micros() - before;

    // digitalRead costs one virtual microsecond per call, so 50 iterations of
    // three buttons is 150. Anything near a millisecond means someone blocked.
    CHECK(spent <= 200);
  }

  return testSummary("buttons");
}
