CXX ?= g++
CXXFLAGS ?= -std=gnu++11 -Wall -Wextra -Werror

ROOT := ../..
BUILD_DIR := build

.PHONY: all test_balance_controller test_drift_controller test_drive_mixer test_robot_state test_bluetooth_control test_sensors clean

all: test_balance_controller test_drift_controller test_drive_mixer test_robot_state test_bluetooth_control test_sensors

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/Arduino.h: Makefile | $(BUILD_DIR)
	printf '#ifndef ARDUINO_H\n#define ARDUINO_H\n#include <stdint.h>\nclass Stream {\npublic:\n  virtual ~Stream() {}\n  virtual int available() = 0;\n  virtual int read() = 0;\n};\n#endif\n' > $@

$(BUILD_DIR)/MeGyro.h: Makefile | $(BUILD_DIR)
	printf '#ifndef ME_GYRO_H\n#define ME_GYRO_H\n#include <stdint.h>\nclass MeGyro {\npublic:\n  explicit MeGyro(uint8_t port) : port_(port) {}\n  void begin() {}\n  void update() {}\n  float getAngleX() const { return angleX; }\n  float getAngleY() const { return angleY; }\n  float getAngleZ() const { return angleZ; }\n  uint8_t port_;\n  static float angleX;\n  static float angleY;\n  static float angleZ;\n};\n#endif\n' > $@

$(BUILD_DIR)/MeUltrasonicSensor.h: Makefile | $(BUILD_DIR)
	printf '#ifndef ME_ULTRASONIC_SENSOR_H\n#define ME_ULTRASONIC_SENSOR_H\n#include <stdint.h>\nclass MeUltrasonicSensor {\npublic:\n  explicit MeUltrasonicSensor(uint8_t port) : port_(port) {}\n  float distanceCm() const { return distance; }\n  uint8_t port_;\n  static float distance;\n};\n#endif\n' > $@

$(BUILD_DIR)/test_balance_controller: test_balance_controller.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp $(ROOT)/SelfBalanceRobot/BalanceController.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_balance_controller.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp -o $@

test_balance_controller: $(BUILD_DIR)/test_balance_controller
	./$(BUILD_DIR)/test_balance_controller

$(BUILD_DIR)/test_drift_controller: test_drift_controller.cpp $(ROOT)/SelfBalanceRobot/DriftController.cpp $(ROOT)/SelfBalanceRobot/DriftController.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_drift_controller.cpp $(ROOT)/SelfBalanceRobot/DriftController.cpp -o $@

test_drift_controller: $(BUILD_DIR)/test_drift_controller
	./$(BUILD_DIR)/test_drift_controller

$(BUILD_DIR)/test_drive_mixer: test_drive_mixer.cpp $(ROOT)/SelfBalanceRobot/DriveMixer.cpp $(ROOT)/SelfBalanceRobot/DriveMixer.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_drive_mixer.cpp $(ROOT)/SelfBalanceRobot/DriveMixer.cpp -o $@

test_drive_mixer: $(BUILD_DIR)/test_drive_mixer
	./$(BUILD_DIR)/test_drive_mixer

$(BUILD_DIR)/test_robot_state: test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp $(ROOT)/SelfBalanceRobot/RobotState.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp -o $@

test_robot_state: $(BUILD_DIR)/test_robot_state
	./$(BUILD_DIR)/test_robot_state

$(BUILD_DIR)/test_bluetooth_control: test_bluetooth_control.cpp $(ROOT)/SelfBalanceRobot/BluetoothControl.cpp $(ROOT)/SelfBalanceRobot/BluetoothControl.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_bluetooth_control.cpp $(ROOT)/SelfBalanceRobot/BluetoothControl.cpp -o $@

test_bluetooth_control: $(BUILD_DIR)/test_bluetooth_control
	./$(BUILD_DIR)/test_bluetooth_control

$(BUILD_DIR)/test_sensors: test_sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h $(BUILD_DIR)/MeGyro.h $(BUILD_DIR)/MeUltrasonicSensor.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.cpp -o $@

test_sensors: $(BUILD_DIR)/test_sensors
	./$(BUILD_DIR)/test_sensors

clean:
	rm -rf $(BUILD_DIR)
