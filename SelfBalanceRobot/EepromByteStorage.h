#ifndef EEPROM_BYTE_STORAGE_H
#define EEPROM_BYTE_STORAGE_H

#include <EEPROM.h>

#include "BalancePointStore.h"

class EepromByteStorage : public BalancePointStorage {
public:
  uint8_t read(uint16_t address) const {
    return EEPROM.read(address);
  }

  void update(uint16_t address, uint8_t value) {
    EEPROM.update(address, value);
  }
};

#endif
