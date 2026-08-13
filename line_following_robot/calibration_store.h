#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H

// Persists the line sensor calibration bounds in EEPROM, so a good sweep
// survives a power cycle.
//
// Why this exists: the robot currently recalibrates on every boot, which
// throws away a careful sweep every time the battery is unplugged and makes
// the run-to-run behaviour depend on how well the operator waved the robot
// over the line in the last three seconds. The 32U4 has 1 KB of EEPROM and
// the whole record is under forty bytes.
//
// The important half of this file is the validation, not the storage. A blank
// or damaged EEPROM must be rejected rather than loaded, because bogus
// calibration bounds do not produce an obviously broken robot: they produce a
// robot that steers confidently on nonsense, which is far harder to diagnose
// than one that has plainly not calibrated at all. So every record carries a
// magic number, a version and a checksum, and load() returns false unless all
// three agree.
//
// No hardware has run any of this. The host tests exercise the layout and the
// rejection logic against a stand-in EEPROM; real cell wear, brown-out during
// a write and the timing of the actual EEPROM driver are all unverified.

#include <stdint.h>

#include <EEPROM.h>

// ------------------------------------------------------------- layout

// Fixed base address. Nothing else in this firmware uses EEPROM yet, so the
// record starts at zero; anything added later must start above
// CALIBRATION_RECORD_BYTES.
static const int CALIBRATION_EEPROM_ADDR = 0;

// Identifies the record as ours. A blank cell reads 0xFF and a half-written
// one reads whatever survived, so an arbitrary two-byte pattern is a cheap
// first filter before the checksum is even computed.
static const uint8_t CALIBRATION_MAGIC_0 = 0x3B;
static const uint8_t CALIBRATION_MAGIC_1 = 0x1C;

// Bump this whenever the byte layout below changes. An old record then fails
// the version check and is discarded, rather than being read as the new layout
// and silently producing wrong bounds.
static const uint8_t CALIBRATION_VERSION = 1;

// The payload is a fixed size regardless of how many sensors are actually
// stored, so the checksum always covers the same span and the record length
// never depends on data that has not been validated yet. Eight leaves room to
// grow past the five sensors fitted today.
static const uint8_t CALIBRATION_MAX_SENSORS = 8;

// Byte offsets from CALIBRATION_EEPROM_ADDR.
static const int CALIBRATION_OFF_MAGIC_0 = 0;
static const int CALIBRATION_OFF_MAGIC_1 = 1;
static const int CALIBRATION_OFF_VERSION = 2;
static const int CALIBRATION_OFF_COUNT   = 3;
static const int CALIBRATION_OFF_PAYLOAD = 4;

// Payload is min[CALIBRATION_MAX_SENSORS] then max[CALIBRATION_MAX_SENSORS],
// each entry two bytes, low byte first. Written byte by byte rather than as a
// struct so the layout does not depend on the compiler's padding or on the
// endianness of whatever machine reads it back.
static const int CALIBRATION_PAYLOAD_BYTES = 2 * 2 * (int)CALIBRATION_MAX_SENSORS;

static const int CALIBRATION_OFF_CHECKSUM   = CALIBRATION_OFF_PAYLOAD + CALIBRATION_PAYLOAD_BYTES;
static const int CALIBRATION_RECORD_BYTES   = CALIBRATION_OFF_CHECKSUM + 2;

class CalibrationStore_c {

  private:

    // Fletcher-16 over the header and the payload. Chosen over a plain sum
    // because it is position sensitive: a sum cannot tell a record apart from
    // the same bytes in a different order, which is exactly the failure a
    // partly overwritten record produces. Still only a few bytes of code.
    uint16_t checksum() const {

      uint8_t sum_a = 0;
      uint8_t sum_b = 0;

      for (int i = 0; i < CALIBRATION_OFF_CHECKSUM; i++) {
        sum_a = (uint8_t)(sum_a + EEPROM.read(CALIBRATION_EEPROM_ADDR + i));
        sum_b = (uint8_t)(sum_b + sum_a);
      }

      return (uint16_t)(((uint16_t)sum_b << 8) | sum_a);

    }

    void writeByte(int offset, uint8_t value) {

      // update(), never write(). An EEPROM cell is good for something like
      // 100k erase/write cycles, and most of a re-saved calibration is
      // usually identical to what is already there. update() compares first
      // and leaves unchanged bytes alone, so repeated saves cost only the
      // bytes that actually moved.
      EEPROM.update(CALIBRATION_EEPROM_ADDR + offset, value);

    }

    void writeWord(int offset, uint16_t value) {

      writeByte(offset,     (uint8_t)(value & 0xFF));
      writeByte(offset + 1, (uint8_t)(value >> 8));

    }

    uint16_t readWord(int offset) const {

      uint16_t lo = EEPROM.read(CALIBRATION_EEPROM_ADDR + offset);
      uint16_t hi = EEPROM.read(CALIBRATION_EEPROM_ADDR + offset + 1);

      return (uint16_t)(lo | (hi << 8));

    }

    // Every reason a record can be refused, in one place, so load() and
    // isPresent() can never disagree about whether the stored data is usable.
    bool headerValid() const {

      if (EEPROM.read(CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_MAGIC_0) != CALIBRATION_MAGIC_0) return false;
      if (EEPROM.read(CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_MAGIC_1) != CALIBRATION_MAGIC_1) return false;
      if (EEPROM.read(CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_VERSION) != CALIBRATION_VERSION) return false;

      uint8_t stored_count = EEPROM.read(CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_COUNT);

      if (stored_count == 0) return false;
      if (stored_count > CALIBRATION_MAX_SENSORS) return false;

      return readWord(CALIBRATION_OFF_CHECKSUM) == checksum();

    }

  public:

    CalibrationStore_c() {}

    // True when EEPROM holds a record that passes magic, version and
    // checksum. Says nothing about whether the bounds are any good, only that
    // they are the bytes that were written.
    bool isPresent() {

      return headerValid();

    }

    // Fills minOut and maxOut from EEPROM and returns true, or leaves both
    // untouched and returns false. The caller must treat false as "no
    // calibration exists" and run a sweep, not as "load some of it anyway".
    bool load(uint16_t *minOut, uint16_t *maxOut, uint8_t count) {

      if (minOut == nullptr || maxOut == nullptr) return false;
      if (count == 0 || count > CALIBRATION_MAX_SENSORS) return false;

      if (!headerValid()) return false;

      // A record written for a different number of sensors is not a partial
      // match to be salvaged. Refusing it is the same argument as the version
      // byte: half a calibration steers worse than none.
      if (EEPROM.read(CALIBRATION_EEPROM_ADDR + CALIBRATION_OFF_COUNT) != count) return false;

      for (uint8_t i = 0; i < count; i++) {
        minOut[i] = readWord(CALIBRATION_OFF_PAYLOAD + 2 * (int)i);
        maxOut[i] = readWord(CALIBRATION_OFF_PAYLOAD + 2 * ((int)CALIBRATION_MAX_SENSORS + (int)i));
      }

      return true;

    }

    // Writes a complete record. Unused payload slots are zeroed rather than
    // left as they were, so the same calibration always produces the same
    // bytes and the checksum is reproducible.
    void save(const uint16_t *min, const uint16_t *max, uint8_t count) {

      if (min == nullptr || max == nullptr) return;
      if (count == 0 || count > CALIBRATION_MAX_SENSORS) return;

      writeByte(CALIBRATION_OFF_MAGIC_0, CALIBRATION_MAGIC_0);
      writeByte(CALIBRATION_OFF_MAGIC_1, CALIBRATION_MAGIC_1);
      writeByte(CALIBRATION_OFF_VERSION, CALIBRATION_VERSION);
      writeByte(CALIBRATION_OFF_COUNT,   count);

      for (uint8_t i = 0; i < CALIBRATION_MAX_SENSORS; i++) {

        uint16_t lo_value = (i < count) ? min[i] : 0;
        uint16_t hi_value = (i < count) ? max[i] : 0;

        writeWord(CALIBRATION_OFF_PAYLOAD + 2 * (int)i, lo_value);
        writeWord(CALIBRATION_OFF_PAYLOAD + 2 * ((int)CALIBRATION_MAX_SENSORS + (int)i), hi_value);

      }

      // Last, and only once the payload is settled: the checksum is what
      // makes the record trustworthy, so it must never be valid before the
      // bytes it covers are in place. A power loss part way through leaves a
      // stale checksum over new payload, which fails validation and is
      // discarded, which is the outcome we want.
      writeWord(CALIBRATION_OFF_CHECKSUM, checksum());

    }

    // Invalidates the record. Only the magic needs to go for the record to be
    // rejected, but the whole header is cleared so nothing half-recognisable
    // is left behind for a future version to puzzle over.
    void clear() {

      writeByte(CALIBRATION_OFF_MAGIC_0, 0xFF);
      writeByte(CALIBRATION_OFF_MAGIC_1, 0xFF);
      writeByte(CALIBRATION_OFF_VERSION, 0xFF);
      writeByte(CALIBRATION_OFF_COUNT,   0xFF);

    }

};

#endif
