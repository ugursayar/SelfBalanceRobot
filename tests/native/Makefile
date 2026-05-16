CXX ?= g++
CXXFLAGS ?= -std=gnu++11 -Wall -Wextra -Werror

ROOT := ../..
BUILD_DIR := build

.PHONY: all test_balance_controller test_drive_mixer test_robot_state clean

all: test_balance_controller test_drive_mixer test_robot_state

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/Arduino.h: | $(BUILD_DIR)
	printf '#include <stdint.h>\n' > $@

$(BUILD_DIR)/test_balance_controller: test_balance_controller.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp $(ROOT)/SelfBalanceRobot/BalanceController.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_balance_controller.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp -o $@

test_balance_controller: $(BUILD_DIR)/test_balance_controller
	./$(BUILD_DIR)/test_balance_controller

$(BUILD_DIR)/test_drive_mixer: test_drive_mixer.cpp $(ROOT)/SelfBalanceRobot/DriveMixer.cpp $(ROOT)/SelfBalanceRobot/DriveMixer.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_drive_mixer.cpp $(ROOT)/SelfBalanceRobot/DriveMixer.cpp -o $@

test_drive_mixer: $(BUILD_DIR)/test_drive_mixer
	./$(BUILD_DIR)/test_drive_mixer

$(BUILD_DIR)/test_robot_state: test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp $(ROOT)/SelfBalanceRobot/RobotState.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp -o $@

test_robot_state: $(BUILD_DIR)/test_robot_state
	./$(BUILD_DIR)/test_robot_state

clean:
	rm -rf $(BUILD_DIR)
