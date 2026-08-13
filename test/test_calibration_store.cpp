// Host tests for calibration_store.h.
//
// Two properties matter more than the round trip, and both are asserted here
// directly rather than inferred:
//
//   1. A record that is absent, stale or damaged is REJECTED. Loading garbage
//      bounds gives a robot that steers confidently on nonsense, which is a
//      far worse failure than one that simply has no calibration. Blank
//      EEPROM, a single flipped payload byte, a damaged checksum, a wrong
//      version and a wrong sensor count are each rejected below.
//
//   2. Saving unchanged data does not rewrite the cells. EEPROM endurance is
//      finite, roughly 100k cycles a cell, and a robot that saves its
//      calibration on every boot would otherwise spend that budget on bytes
//      that never changed. The mock counts writes, so this is checked rather
//      than assumed.

#include "arduino_stub.h"
#include "test_harness.h"

#include "calibration_store.h"

// Five sensors today, but the store is deliberately independent of the
// firmware's NUM_SENSORS, so the tests declare their own.
static const uint8_t COUNT = 5;

// Power-cycle the virtual robot: EEPROM keeps its contents, everything held
// in RAM does not.
static void freshEeprom() {
  mockEepromClear();
  mockEepromWrites = 0;
}

int main() {
  printf("calibration_store\n");

  const uint16_t min_in[COUNT] = { 120, 133, 98, 141, 127 };
  const uint16_t max_in[COUNT] = { 1900, 2100, 2050, 1980, 2200 };

  {
    TEST("the record fits in the 32U4 EEPROM and leaves room to spare");
    CHECK(CALIBRATION_EEPROM_ADDR >= 0);
    CHECK(CALIBRATION_RECORD_BYTES > 0);
    CHECK((size_t)(CALIBRATION_EEPROM_ADDR + CALIBRATION_RECORD_BYTES) <= MOCK_EEPROM_SIZE);
    CHECK(COUNT <= CALIBRATION_MAX_SENSORS);
  }

  {
    // EEPROM ships as 0xFF. Reading that as a calibration would give bounds
    // of 65535 on every sensor and a robot that never sees a line.
    TEST("blank EEPROM is rejected, not read as calibration");
    freshEeprom();
    CalibrationStore_c store;

    uint16_t min_out[COUNT], max_out[COUNT];
    for (uint8_t i = 0; i < COUNT; i++) { min_out[i] = 0xAAAA; max_out[i] = 0x5555; }

    CHECK(!store.isPresent());
    CHECK(!store.load(min_out, max_out, COUNT));

    // and the caller's buffers are left exactly as they were
    for (uint8_t i = 0; i < COUNT; i++) {
      CHECK_EQ(min_out[i], 0xAAAA);
      CHECK_EQ(max_out[i], 0x5555);
    }
  }

  {
    TEST("an all-zero EEPROM is rejected too");
    freshEeprom();
    for (size_t i = 0; i < MOCK_EEPROM_SIZE; i++) mockEepromBytes[i] = 0x00;
    CalibrationStore_c store;

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(!store.isPresent());
    CHECK(!store.load(min_out, max_out, COUNT));
  }

  {
    TEST("a saved calibration reads back exactly");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);

    CHECK(store.isPresent());

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(store.load(min_out, max_out, COUNT));

    for (uint8_t i = 0; i < COUNT; i++) {
      CHECK_EQ(min_out[i], min_in[i]);
      CHECK_EQ(max_out[i], max_in[i]);
    }
  }

  {
    // The reason this file exists: a sweep must outlive the battery being
    // unplugged. A second store object reads the same cells with no state
    // carried over in RAM.
    TEST("the record survives a power cycle");
    freshEeprom();
    {
      CalibrationStore_c writer;
      writer.save(min_in, max_in, COUNT);
    }

    CalibrationStore_c reader;  // as if after a reset
    uint16_t min_out[COUNT], max_out[COUNT];

    CHECK(reader.isPresent());
    CHECK(reader.load(min_out, max_out, COUNT));
    CHECK_EQ(min_out[0], min_in[0]);
    CHECK_EQ(max_out[COUNT - 1], max_in[COUNT - 1]);
  }

  {
    // One byte of the payload corrupted, everything else intact. The magic
    // and version still match, so only the checksum can catch this.
    TEST("a single flipped payload byte is rejected");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);
    CHECK(store.isPresent());

    int victim = CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_PAYLOAD + 3;
    mockEepromBytes[victim] ^= 0x01;  // one bit, in one byte

    uint16_t min_out[COUNT], max_out[COUNT];
    for (uint8_t i = 0; i < COUNT; i++) { min_out[i] = 0xAAAA; max_out[i] = 0x5555; }

    CHECK(!store.isPresent());
    CHECK(!store.load(min_out, max_out, COUNT));
    CHECK_EQ(min_out[0], 0xAAAA);  // nothing partially loaded before the check
    CHECK_EQ(max_out[0], 0x5555);
  }

  {
    // Every byte the checksum covers, one at a time. A checksum that only
    // caught damage in some positions would be worse than useless, because it
    // would still look like it was doing its job.
    TEST("a flip anywhere in the covered record is rejected");
    for (int offset = 0; offset < CALIBRATION_OFF_CHECKSUM; offset++) {
      freshEeprom();
      CalibrationStore_c store;
      store.save(min_in, max_in, COUNT);

      mockEepromBytes[CALIBRATION_EEPROM_ADDR + offset] ^= 0x40;

      uint16_t min_out[COUNT], max_out[COUNT];
      CHECK(!store.load(min_out, max_out, COUNT));
    }
  }

  {
    TEST("a damaged checksum field is rejected");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);

    mockEepromBytes[CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_CHECKSUM] ^= 0xFF;

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(!store.isPresent());
    CHECK(!store.load(min_out, max_out, COUNT));
  }

  {
    // Simulates a record written by an earlier layout: the checksum is
    // recomputed so it is internally consistent, and only the version byte
    // says it is not ours to read.
    TEST("a record from a different version is rejected, not misread");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);

    mockEepromBytes[CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_VERSION] = CALIBRATION_VERSION + 1;

    // recompute a valid Fletcher-16 over the altered record
    uint8_t sum_a = 0, sum_b = 0;
    for (int i = 0; i < CALIBRATION_OFF_CHECKSUM; i++) {
      sum_a = (uint8_t)(sum_a + mockEepromBytes[CALIBRATION_EEPROM_ADDR + i]);
      sum_b = (uint8_t)(sum_b + sum_a);
    }
    mockEepromBytes[CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_CHECKSUM]     = sum_a;
    mockEepromBytes[CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_CHECKSUM + 1] = sum_b;

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(!store.isPresent());
    CHECK(!store.load(min_out, max_out, COUNT));
  }

  {
    TEST("a wrong magic is rejected");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);

    mockEepromBytes[CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_MAGIC_0] = 0x00;

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(!store.isPresent());
    CHECK(!store.load(min_out, max_out, COUNT));
  }

  {
    // A four-sensor record is not half of a five-sensor one. Salvaging it
    // would leave the fifth sensor on fallback bounds while the others use
    // calibrated ones, which biases the steering.
    TEST("a record written for a different sensor count is refused");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, 4);

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(store.isPresent());              // the record itself is sound
    CHECK(!store.load(min_out, max_out, COUNT));  // just not for five sensors
    CHECK(store.load(min_out, max_out, 4));       // and still loads for four
  }

  {
    TEST("clear invalidates the record");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);
    CHECK(store.isPresent());

    store.clear();

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(!store.isPresent());
    CHECK(!store.load(min_out, max_out, COUNT));
  }

  {
    // The endurance assertion. EEPROM.update() compares before writing, so
    // re-saving identical data must cost nothing at all.
    TEST("saving identical data twice does not rewrite any cell");
    freshEeprom();
    CalibrationStore_c store;

    store.save(min_in, max_in, COUNT);
    int after_first = mockEepromWrites;
    CHECK(after_first > 0);  // the first save really did write

    store.save(min_in, max_in, COUNT);
    CHECK_EQ(mockEepromWrites, after_first);  // the second wrote nothing

    store.save(min_in, max_in, COUNT);
    store.save(min_in, max_in, COUNT);
    CHECK_EQ(mockEepromWrites, after_first);

    // and the record is still intact after all that
    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(store.load(min_out, max_out, COUNT));
    CHECK_EQ(min_out[2], min_in[2]);
  }

  {
    TEST("only the bytes that changed are rewritten");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);

    uint16_t min_changed[COUNT];
    for (uint8_t i = 0; i < COUNT; i++) min_changed[i] = min_in[i];
    min_changed[0] = (uint16_t)(min_in[0] + 1);  // differs in the low byte only

    mockEepromWrites = 0;
    store.save(min_changed, max_in, COUNT);

    // one payload byte, plus at most the two checksum bytes
    CHECK(mockEepromWrites > 0);
    CHECK(mockEepromWrites <= 3);

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(store.load(min_out, max_out, COUNT));
    CHECK_EQ(min_out[0], min_changed[0]);
  }

  {
    TEST("a nonsensical count is refused rather than written");
    freshEeprom();
    CalibrationStore_c store;

    store.save(min_in, max_in, 0);
    CHECK_EQ(mockEepromWrites, 0);
    CHECK(!store.isPresent());

    store.save(min_in, max_in, (uint8_t)(CALIBRATION_MAX_SENSORS + 1));
    CHECK_EQ(mockEepromWrites, 0);
    CHECK(!store.isPresent());

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(!store.load(min_out, max_out, 0));
    CHECK(!store.load(min_out, max_out, (uint8_t)(CALIBRATION_MAX_SENSORS + 1)));
  }

  {
    TEST("a null destination is refused");
    freshEeprom();
    CalibrationStore_c store;
    store.save(min_in, max_in, COUNT);

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(!store.load(nullptr, max_out, COUNT));
    CHECK(!store.load(min_out, nullptr, COUNT));

    mockEepromWrites = 0;
    store.save(nullptr, max_in, COUNT);
    store.save(min_in, nullptr, COUNT);
    CHECK_EQ(mockEepromWrites, 0);
  }

  {
    // The extreme values a sensor can report, to catch any byte order or
    // truncation mistake that mid-range numbers would hide.
    TEST("boundary values round trip intact");
    freshEeprom();
    CalibrationStore_c store;

    const uint16_t lo[COUNT] = { 0, 1, 255, 256, 0xFFFE };
    const uint16_t hi[COUNT] = { 0xFFFF, 0xFF00, 0x00FF, 0x0100, 0xFFFF };
    store.save(lo, hi, COUNT);

    uint16_t min_out[COUNT], max_out[COUNT];
    CHECK(store.load(min_out, max_out, COUNT));
    for (uint8_t i = 0; i < COUNT; i++) {
      CHECK_EQ(min_out[i], lo[i]);
      CHECK_EQ(max_out[i], hi[i]);
    }
  }

  return testSummary("calibration_store");
}
