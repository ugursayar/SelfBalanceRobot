CXX ?= g++
CXXFLAGS ?= -std=gnu++11 -Wall -Wextra -Werror

ROOT := ../..
BUILD_DIR := build

.PHONY: all test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner clean

all: test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/Arduino.h: Makefile | $(BUILD_DIR)
	printf '#ifndef ARDUINO_H\n#define ARDUINO_H\n#include <stdint.h>\nclass Stream {\npublic:\n  virtual ~Stream() {}\n  virtual int available() = 0;\n  virtual int read() = 0;\n};\n#endif\n' > $@

$(BUILD_DIR)/MeGyro.h: Makefile | $(BUILD_DIR)
	printf '#ifndef ME_GYRO_H\n#define ME_GYRO_H\n#include <stdint.h>\nclass MeGyro {\npublic:\n  explicit MeGyro(uint8_t port) : port_(port) {}\n  void begin() {}\n  void update() {}\n  float getAngleX() const { return angleX; }\n  float getAngleY() const { return angleY; }\n  float getAngleZ() const { return angleZ; }\n  float getGyroX() const { return gyroX; }\n  float getGyroY() const { return gyroY; }\n  float getGyroZ() const { return gyroZ; }\n  uint8_t port_;\n  static float angleX;\n  static float angleY;\n  static float angleZ;\n  static float gyroX;\n  static float gyroY;\n  static float gyroZ;\n};\n#endif\n' > $@

$(BUILD_DIR)/test_auto_arm_controller: test_auto_arm_controller.cpp $(ROOT)/SelfBalanceRobot/AutoArmController.cpp $(ROOT)/SelfBalanceRobot/AutoArmController.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_auto_arm_controller.cpp $(ROOT)/SelfBalanceRobot/AutoArmController.cpp -o $@

test_auto_arm_controller: $(BUILD_DIR)/test_auto_arm_controller
	./$(BUILD_DIR)/test_auto_arm_controller

$(BUILD_DIR)/test_balance_controller: test_balance_controller.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp $(ROOT)/SelfBalanceRobot/BalanceController.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_balance_controller.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp -o $@

test_balance_controller: $(BUILD_DIR)/test_balance_controller
	./$(BUILD_DIR)/test_balance_controller

$(BUILD_DIR)/test_balance_point_store: test_balance_point_store.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_balance_point_store.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.cpp -o $@

test_balance_point_store: $(BUILD_DIR)/test_balance_point_store
	./$(BUILD_DIR)/test_balance_point_store

$(BUILD_DIR)/test_balance_point_learner: test_balance_point_learner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_balance_point_learner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.cpp -o $@

test_balance_point_learner: $(BUILD_DIR)/test_balance_point_learner
	./$(BUILD_DIR)/test_balance_point_learner

$(BUILD_DIR)/test_robot_state: test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp $(ROOT)/SelfBalanceRobot/RobotState.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp -o $@

test_robot_state: $(BUILD_DIR)/test_robot_state
	./$(BUILD_DIR)/test_robot_state

$(BUILD_DIR)/test_sensors: test_sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h $(BUILD_DIR)/MeGyro.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.cpp -o $@

test_sensors: $(BUILD_DIR)/test_sensors
	./$(BUILD_DIR)/test_sensors

clean:
	rm -rf $(BUILD_DIR)
