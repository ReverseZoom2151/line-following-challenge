// Host stand-in for the Arduino EEPROM library.
//
// Firmware writes `#include <EEPROM.h>`, and the test build puts test/ on the
// include path, so this file satisfies that include when compiling on a host.
// It is backed by a plain array, so a test can power-cycle the virtual robot
// by calling mockEepromClear() and confirm that stored calibration survives.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// The ATmega32U4 has 1 KB of EEPROM.
static const size_t MOCK_EEPROM_SIZE = 1024;

inline uint8_t mockEepromBytes[MOCK_EEPROM_SIZE] = {};

// EEPROM ships from the factory as 0xFF, and a fresh chip must not be mistaken
// for one holding a valid record.
inline void mockEepromClear() {
  memset(mockEepromBytes, 0xFF, MOCK_EEPROM_SIZE);
}

inline int mockEepromWrites = 0;

class MockEEPROM {
 public:
  uint8_t read(int address) const {
    if (address < 0 || (size_t)address >= MOCK_EEPROM_SIZE) return 0xFF;
    return mockEepromBytes[address];
  }

  void write(int address, uint8_t value) {
    if (address < 0 || (size_t)address >= MOCK_EEPROM_SIZE) return;
    mockEepromBytes[address] = value;
    mockEepromWrites++;
  }

  // update() only writes when the value differs, which is how real firmware
  // avoids burning through the cell's limited write endurance.
  void update(int address, uint8_t value) {
    if (address < 0 || (size_t)address >= MOCK_EEPROM_SIZE) return;
    if (mockEepromBytes[address] != value) write(address, value);
  }

  template <typename T>
  const T &put(int address, const T &value) {
    const uint8_t *src = reinterpret_cast<const uint8_t *>(&value);
    for (size_t i = 0; i < sizeof(T); i++) update(address + (int)i, src[i]);
    return value;
  }

  template <typename T>
  T &get(int address, T &value) {
    uint8_t *dst = reinterpret_cast<uint8_t *>(&value);
    for (size_t i = 0; i < sizeof(T); i++) dst[i] = read(address + (int)i);
    return value;
  }

  size_t length() const { return MOCK_EEPROM_SIZE; }
};

inline MockEEPROM EEPROM;
