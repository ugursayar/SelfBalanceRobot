CXX ?= g++
CXXFLAGS ?= -std=gnu++11 -Wall -Wextra -Werror

ROOT := ../..
BUILD_DIR := build

.PHONY: all test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner test_command_parser test_command_reader test_bluetooth_serial_port test_config test_reset_diagnostics test_runtime_stats test_balance_pipeline test_motor_output_latch test_feedback_policy test_telemetry_formatter clean

all: test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner test_command_parser test_command_reader test_bluetooth_serial_port test_config test_reset_diagnostics test_runtime_stats test_balance_pipeline test_motor_output_latch test_feedback_policy test_telemetry_formatter

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

$(BUILD_DIR)/test_balance_pipeline: test_balance_pipeline.cpp $(ROOT)/SelfBalanceRobot/BalancePipeline.cpp $(ROOT)/SelfBalanceRobot/BalancePipeline.h $(ROOT)/SelfBalanceRobot/BalanceController.cpp $(ROOT)/SelfBalanceRobot/BalanceController.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_balance_pipeline.cpp $(ROOT)/SelfBalanceRobot/BalancePipeline.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp -o $@

test_balance_pipeline: $(BUILD_DIR)/test_balance_pipeline
	./$(BUILD_DIR)/test_balance_pipeline

$(BUILD_DIR)/test_balance_point_store: test_balance_point_store.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_balance_point_store.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.cpp -o $@

test_balance_point_store: $(BUILD_DIR)/test_balance_point_store
	./$(BUILD_DIR)/test_balance_point_store

$(BUILD_DIR)/test_balance_point_learner: test_balance_point_learner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_balance_point_learner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.cpp -o $@

test_balance_point_learner: $(BUILD_DIR)/test_balance_point_learner
	./$(BUILD_DIR)/test_balance_point_learner

$(BUILD_DIR)/test_command_parser: test_command_parser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_command_parser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.cpp -o $@

test_command_parser: $(BUILD_DIR)/test_command_parser
	./$(BUILD_DIR)/test_command_parser

$(BUILD_DIR)/test_command_reader: test_command_reader.cpp $(ROOT)/SelfBalanceRobot/CommandReader.cpp $(ROOT)/SelfBalanceRobot/CommandReader.h $(ROOT)/SelfBalanceRobot/CommandParser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_command_reader.cpp $(ROOT)/SelfBalanceRobot/CommandReader.cpp $(ROOT)/SelfBalanceRobot/CommandParser.cpp -o $@

test_command_reader: $(BUILD_DIR)/test_command_reader
	./$(BUILD_DIR)/test_command_reader

$(BUILD_DIR)/test_bluetooth_serial_port: test_bluetooth_serial_port.cpp $(ROOT)/SelfBalanceRobot/BluetoothSerialPort.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_bluetooth_serial_port.cpp -o $@

test_bluetooth_serial_port: $(BUILD_DIR)/test_bluetooth_serial_port
	./$(BUILD_DIR)/test_bluetooth_serial_port

$(BUILD_DIR)/test_feedback_policy: test_feedback_policy.cpp $(ROOT)/SelfBalanceRobot/FeedbackPolicy.cpp $(ROOT)/SelfBalanceRobot/FeedbackPolicy.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_feedback_policy.cpp $(ROOT)/SelfBalanceRobot/FeedbackPolicy.cpp -o $@

test_feedback_policy: $(BUILD_DIR)/test_feedback_policy
	./$(BUILD_DIR)/test_feedback_policy

$(BUILD_DIR)/test_motor_output_latch: test_motor_output_latch.cpp $(ROOT)/SelfBalanceRobot/MotorOutputLatch.cpp $(ROOT)/SelfBalanceRobot/MotorOutputLatch.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_motor_output_latch.cpp $(ROOT)/SelfBalanceRobot/MotorOutputLatch.cpp -o $@

test_motor_output_latch: $(BUILD_DIR)/test_motor_output_latch
	./$(BUILD_DIR)/test_motor_output_latch

$(BUILD_DIR)/test_config: test_config.cpp $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_config.cpp -o $@

test_config: $(BUILD_DIR)/test_config
	./$(BUILD_DIR)/test_config

$(BUILD_DIR)/test_reset_diagnostics: test_reset_diagnostics.cpp $(ROOT)/SelfBalanceRobot/ResetDiagnostics.cpp $(ROOT)/SelfBalanceRobot/ResetDiagnostics.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_reset_diagnostics.cpp $(ROOT)/SelfBalanceRobot/ResetDiagnostics.cpp -o $@

test_reset_diagnostics: $(BUILD_DIR)/test_reset_diagnostics
	./$(BUILD_DIR)/test_reset_diagnostics

$(BUILD_DIR)/test_runtime_stats: test_runtime_stats.cpp $(ROOT)/SelfBalanceRobot/RuntimeStats.cpp $(ROOT)/SelfBalanceRobot/RuntimeStats.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_runtime_stats.cpp $(ROOT)/SelfBalanceRobot/RuntimeStats.cpp -o $@

test_runtime_stats: $(BUILD_DIR)/test_runtime_stats
	./$(BUILD_DIR)/test_runtime_stats

$(BUILD_DIR)/test_robot_state: test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp $(ROOT)/SelfBalanceRobot/RobotState.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_robot_state.cpp $(ROOT)/SelfBalanceRobot/RobotState.cpp -o $@

test_robot_state: $(BUILD_DIR)/test_robot_state
	./$(BUILD_DIR)/test_robot_state

$(BUILD_DIR)/test_sensors: test_sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h $(BUILD_DIR)/MeGyro.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_sensors.cpp $(ROOT)/SelfBalanceRobot/Sensors.cpp -o $@

test_sensors: $(BUILD_DIR)/test_sensors
	./$(BUILD_DIR)/test_sensors

$(BUILD_DIR)/test_telemetry_formatter: test_telemetry_formatter.cpp $(ROOT)/SelfBalanceRobot/TelemetryFormatter.h $(ROOT)/SelfBalanceRobot/RuntimeStats.h $(ROOT)/SelfBalanceRobot/ResetDiagnostics.cpp $(ROOT)/SelfBalanceRobot/ResetDiagnostics.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_telemetry_formatter.cpp $(ROOT)/SelfBalanceRobot/ResetDiagnostics.cpp -o $@

test_telemetry_formatter: $(BUILD_DIR)/test_telemetry_formatter
	./$(BUILD_DIR)/test_telemetry_formatter

clean:
	rm -rf $(BUILD_DIR)
