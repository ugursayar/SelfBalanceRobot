#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "../../SelfBalanceRobot/BalancePointStore.h"

class FakeStorage : public BalancePointStorage {
public:
  FakeStorage() : bytes_(128, 0xFF) {}

  uint8_t read(uint16_t address) const {
    return bytes_[address];
  }

  void update(uint16_t address, uint8_t value) {
    bytes_[address] = value;
  }

  void corrupt(uint16_t address) {
    bytes_[address] ^= 0x55;
  }

private:
  std::vector<uint8_t> bytes_;
};

static const uint16_t kSlotSize = 13;

static BalancePointStore storeFor(FakeStorage& storage,
                                  uint16_t baseAddress = 0) {
  BalancePointStore store(storage, baseAddress);
  store.configure(-12.0f, 12.0f);
  return store;
}

static void test_empty_eeprom_uses_fallback() {
  FakeStorage storage;
  BalancePointStore store = storeFor(storage);

  assert(!store.begin(0.7f));
  assert(!store.hasStoredBalancePoint());
  assert(store.balancePointDegrees() == 0.7f);
}

static void test_save_and_reload_valid_balance_point() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(1.25f));
  assert(writer.hasStoredBalancePoint());
  assert(writer.balancePointDegrees() == 1.25f);
  assert(writer.writeCounter() == 1);

  BalancePointStore reader = storeFor(storage);
  assert(reader.begin(0.0f));
  assert(reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 1.25f);
  assert(reader.writeCounter() == 1);
}

static void test_invalid_checksum_is_rejected() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(-0.5f));

  storage.corrupt(5);

  BalancePointStore reader = storeFor(storage);
  assert(!reader.begin(0.7f));
  assert(!reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 0.7f);
}

static void test_multiple_saves_reload_newest_value() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(1.0f));
  assert(writer.saveBalancePoint(2.0f));
  assert(writer.saveBalancePoint(3.0f));
  assert(writer.writeCounter() == 3);

  BalancePointStore reader = storeFor(storage);
  assert(reader.begin(0.0f));
  assert(reader.balancePointDegrees() == 3.0f);
  assert(reader.writeCounter() == 3);
}

static void test_corrupt_newest_slot_falls_back_to_previous_valid_slot() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(1.0f));
  assert(writer.saveBalancePoint(2.0f));

  storage.corrupt(kSlotSize + 5);

  BalancePointStore reader = storeFor(storage);
  assert(reader.begin(0.0f));
  assert(reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 1.0f);
  assert(reader.writeCounter() == 1);
}

static void test_invalid_save_preserves_existing_valid_record() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(1.25f));

  assert(!writer.saveBalancePoint(20.0f));
  assert(writer.hasStoredBalancePoint());
  assert(writer.balancePointDegrees() == 1.25f);
  assert(writer.writeCounter() == 1);

  BalancePointStore reader = storeFor(storage);
  assert(reader.begin(0.0f));
  assert(reader.balancePointDegrees() == 1.25f);
  assert(reader.writeCounter() == 1);
}

static void test_nonzero_base_address_works() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage, 17);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(-2.5f));

  BalancePointStore reader = storeFor(storage, 17);
  assert(reader.begin(0.0f));
  assert(reader.balancePointDegrees() == -2.5f);
  assert(reader.writeCounter() == 1);

  BalancePointStore wrongBaseReader = storeFor(storage, 0);
  assert(!wrongBaseReader.begin(0.7f));
  assert(wrongBaseReader.balancePointDegrees() == 0.7f);
}

static void test_stored_out_of_range_record_is_rejected() {
  FakeStorage storage;
  BalancePointStore wideWriter(storage, 0);
  wideWriter.configure(-90.0f, 90.0f);
  assert(!wideWriter.begin(0.7f));
  assert(wideWriter.saveBalancePoint(30.0f));

  BalancePointStore reader = storeFor(storage);
  assert(!reader.begin(0.7f));
  assert(!reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 0.7f);
}

static void test_out_of_range_values_are_not_saved() {
  FakeStorage storage;
  BalancePointStore store = storeFor(storage);
  assert(!store.begin(0.7f));

  assert(!store.saveBalancePoint(20.0f));
  assert(!store.hasStoredBalancePoint());
  assert(store.balancePointDegrees() == 0.7f);
}

static void test_clear_invalidates_records_and_uses_fallback() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(1.25f));
  assert(writer.saveBalancePoint(2.5f));

  writer.clearBalancePoint(0.7f);
  assert(!writer.hasStoredBalancePoint());
  assert(writer.balancePointDegrees() == 0.7f);
  assert(writer.writeCounter() == 0);

  BalancePointStore reader = storeFor(storage);
  assert(!reader.begin(0.7f));
  assert(!reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 0.7f);
  assert(reader.writeCounter() == 0);
}

int main() {
  test_empty_eeprom_uses_fallback();
  test_save_and_reload_valid_balance_point();
  test_invalid_checksum_is_rejected();
  test_multiple_saves_reload_newest_value();
  test_corrupt_newest_slot_falls_back_to_previous_valid_slot();
  test_invalid_save_preserves_existing_valid_record();
  test_nonzero_base_address_works();
  test_stored_out_of_range_record_is_rejected();
  test_out_of_range_values_are_not_saved();
  test_clear_invalidates_records_and_uses_fallback();

  std::cout << "test_balance_point_store PASS\n";
  return EXIT_SUCCESS;
}
