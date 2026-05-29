#include "BalancePointStore.h"

namespace {

void writeFloatBytes(uint8_t* bytes, float value) {
  union {
    float value;
    uint8_t bytes[sizeof(float)];
  } converter;

  converter.value = value;
  for (uint8_t i = 0; i < sizeof(float); ++i) {
    bytes[3 + i] = converter.bytes[i];
  }
}

float readFloatBytes(const uint8_t* bytes) {
  union {
    float value;
    uint8_t bytes[sizeof(float)];
  } converter;

  for (uint8_t i = 0; i < sizeof(float); ++i) {
    converter.bytes[i] = bytes[3 + i];
  }
  return converter.value;
}

void writeUint32Bytes(uint8_t* bytes, uint32_t value) {
  bytes[7] = static_cast<uint8_t>(value & 0xFF);
  bytes[8] = static_cast<uint8_t>((value >> 8) & 0xFF);
  bytes[9] = static_cast<uint8_t>((value >> 16) & 0xFF);
  bytes[10] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

uint32_t readUint32Bytes(const uint8_t* bytes) {
  return static_cast<uint32_t>(bytes[7]) |
         (static_cast<uint32_t>(bytes[8]) << 8) |
         (static_cast<uint32_t>(bytes[9]) << 16) |
         (static_cast<uint32_t>(bytes[10]) << 24);
}

}  // namespace

BalancePointStore::BalancePointStore(BalancePointStorage& storage,
                                     uint16_t baseAddress)
    : storage_(storage),
      baseAddress_(baseAddress),
      minimumDegrees_(-180.0f),
      maximumDegrees_(180.0f),
      balancePointDegrees_(0.0f),
      writeCounter_(0),
      hasStoredBalancePoint_(false),
      activeSlot_(kNoActiveSlot) {}

void BalancePointStore::configure(float minimumDegrees, float maximumDegrees) {
  minimumDegrees_ = minimumDegrees;
  maximumDegrees_ = maximumDegrees;
}

bool BalancePointStore::begin(float fallbackDegrees) {
  bool foundRecord = false;
  float newestAngle = fallbackDegrees;
  uint32_t newestCounter = 0;
  uint8_t newestSlot = kNoActiveSlot;

  for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
    float slotAngle = fallbackDegrees;
    uint32_t slotCounter = 0;
    if (readSlot(slot, slotAngle, slotCounter) &&
        (!foundRecord || slotCounter > newestCounter)) {
      foundRecord = true;
      newestAngle = slotAngle;
      newestCounter = slotCounter;
      newestSlot = slot;
    }
  }

  if (foundRecord) {
    balancePointDegrees_ = newestAngle;
    writeCounter_ = newestCounter;
    hasStoredBalancePoint_ = true;
    activeSlot_ = newestSlot;
    return true;
  }

  balancePointDegrees_ = fallbackDegrees;
  writeCounter_ = 0;
  hasStoredBalancePoint_ = false;
  activeSlot_ = kNoActiveSlot;
  return false;
}

bool BalancePointStore::hasStoredBalancePoint() const {
  return hasStoredBalancePoint_;
}

float BalancePointStore::balancePointDegrees() const {
  return balancePointDegrees_;
}

uint32_t BalancePointStore::writeCounter() const {
  return writeCounter_;
}

bool BalancePointStore::saveBalancePoint(float angleDegrees) {
  if (!angleInRange(angleDegrees)) {
    return false;
  }

  const uint32_t nextCounter = writeCounter_ + 1;
  const uint8_t nextSlot = inactiveSlot();
  writeSlot(nextSlot, angleDegrees, nextCounter);
  balancePointDegrees_ = angleDegrees;
  writeCounter_ = nextCounter;
  hasStoredBalancePoint_ = true;
  activeSlot_ = nextSlot;
  return true;
}

bool BalancePointStore::angleInRange(float angleDegrees) const {
  return angleDegrees >= minimumDegrees_ && angleDegrees <= maximumDegrees_;
}

bool BalancePointStore::readSlot(uint8_t slot, float& angleDegrees,
                                 uint32_t& counter) const {
  uint8_t bytes[kRecordSize];
  const uint16_t offset = slotOffset(slot);
  for (uint8_t i = 0; i < kRecordSize; ++i) {
    bytes[i] = readByte(static_cast<uint16_t>(offset + i));
  }

  if (bytes[0] != kMagic0 || bytes[1] != kMagic1 || bytes[2] != kVersion) {
    return false;
  }

  const uint16_t storedCrc = static_cast<uint16_t>(bytes[kCrcLowOffset]) |
                             (static_cast<uint16_t>(bytes[kCrcHighOffset])
                              << 8);
  if (crc16(bytes, kCrcLowOffset) != storedCrc) {
    return false;
  }

  const float storedAngle = readFloatBytes(bytes);
  if (!angleInRange(storedAngle)) {
    return false;
  }

  angleDegrees = storedAngle;
  counter = readUint32Bytes(bytes);
  return true;
}

void BalancePointStore::writeSlot(uint8_t slot, float angleDegrees,
                                  uint32_t counter) {
  uint8_t bytes[kRecordSize];
  const uint16_t offset = slotOffset(slot);
  bytes[0] = kMagic0;
  bytes[1] = kMagic1;
  bytes[2] = kVersion;
  writeFloatBytes(bytes, angleDegrees);
  writeUint32Bytes(bytes, counter);
  const uint16_t crc = crc16(bytes, kCrcLowOffset);
  bytes[kCrcLowOffset] = static_cast<uint8_t>(crc & 0xFF);
  bytes[kCrcHighOffset] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  for (uint8_t i = 0; i < kCrcHighOffset; ++i) {
    writeByte(static_cast<uint16_t>(offset + i), bytes[i]);
  }
  writeByte(static_cast<uint16_t>(offset + kCrcHighOffset),
            bytes[kCrcHighOffset]);
}

uint16_t BalancePointStore::crc16(const uint8_t* bytes, uint8_t length) const {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(bytes[i]);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

uint16_t BalancePointStore::slotOffset(uint8_t slot) const {
  return static_cast<uint16_t>(slot * kRecordSize);
}

uint8_t BalancePointStore::inactiveSlot() const {
  if (activeSlot_ == 0) {
    return 1;
  }
  return 0;
}

uint8_t BalancePointStore::readByte(uint16_t offset) const {
  return storage_.read(static_cast<uint16_t>(baseAddress_ + offset));
}

void BalancePointStore::writeByte(uint16_t offset, uint8_t value) {
  storage_.update(static_cast<uint16_t>(baseAddress_ + offset), value);
}
