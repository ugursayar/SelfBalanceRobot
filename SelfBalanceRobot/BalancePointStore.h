#ifndef BALANCE_POINT_STORE_H
#define BALANCE_POINT_STORE_H

#include <stdint.h>

class BalancePointStorage {
public:
  virtual ~BalancePointStorage() {}
  virtual uint8_t read(uint16_t address) const = 0;
  virtual void update(uint16_t address, uint8_t value) = 0;
};

class BalancePointStore {
public:
  BalancePointStore(BalancePointStorage& storage, uint16_t baseAddress);

  void configure(float minimumDegrees, float maximumDegrees);
  bool begin(float fallbackDegrees);
  bool hasStoredBalancePoint() const;
  float balancePointDegrees() const;
  uint32_t writeCounter() const;
  bool saveBalancePoint(float angleDegrees);

private:
  static const uint8_t kMagic0 = 0x42;
  static const uint8_t kMagic1 = 0x50;
  static const uint8_t kVersion = 1;
  static const uint8_t kRecordSize = 13;
  static const uint8_t kCrcLowOffset = 11;
  static const uint8_t kCrcHighOffset = 12;
  static const uint8_t kSlotCount = 2;
  static const uint8_t kNoActiveSlot = 0xFF;

  bool angleInRange(float angleDegrees) const;
  bool readSlot(uint8_t slot, float& angleDegrees, uint32_t& counter) const;
  void writeSlot(uint8_t slot, float angleDegrees, uint32_t counter);
  uint16_t crc16(const uint8_t* bytes, uint8_t length) const;
  uint16_t slotOffset(uint8_t slot) const;
  uint8_t inactiveSlot() const;
  uint8_t readByte(uint16_t offset) const;
  void writeByte(uint16_t offset, uint8_t value);

  BalancePointStorage& storage_;
  uint16_t baseAddress_;
  float minimumDegrees_;
  float maximumDegrees_;
  float balancePointDegrees_;
  uint32_t writeCounter_;
  bool hasStoredBalancePoint_;
  uint8_t activeSlot_;
};

#endif
