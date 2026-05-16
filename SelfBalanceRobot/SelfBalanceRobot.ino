#include "BalanceController.h"
#include "BluetoothControl.h"
#include "DriveMixer.h"
#include "Motors.h"
#include "RobotState.h"
#include "Sensors.h"
#include "config.h"

#include <MeMegaPi.h>

Sensors sensors;
Motors motors;
BluetoothControl bluetooth;
BalanceController balance;
DriveMixer mixer;
RobotState robotState;

unsigned long lastBalanceMicros = 0;
unsigned long lastDebugMillis = 0;
SensorFrame lastFrame;
ControlCommand lastCommand;
MotorCommand lastMotorOutput;

void printDebug(const SensorFrame& frame, const ControlCommand& command,
                const MotorCommand& motorOutput);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  sensors.begin();
  motors.begin();
  bluetooth.begin(Serial1);

  balance.setTunings(Config::BalanceKp, Config::BalanceKi, Config::BalanceKd);
  balance.setOutputLimit(Config::MaxMotorCommand);

  mixer.setLimits(Config::MaxMotorCommand, Config::MaxDriveCommand,
                  Config::MaxTurnCommand, Config::MotorDeadband);

  robotState.configure(Config::FallAngleDegrees,
                       Config::StillAngleDeltaDegrees,
                       Config::ObstacleStopDistanceCm,
                       Config::CalibrationMillis,
                       Config::CommandTimeoutMillis);

  lastBalanceMicros = micros();

  if (Config::EnableDebugSerial) {
    Serial.println(F("SelfBalanceRobot ready"));
  }
}

void loop() {
  const unsigned long nowMicros = micros();
  const unsigned long elapsedMicros = nowMicros - lastBalanceMicros;
  if (elapsedMicros < Config::BalanceLoopMicros) {
    return;
  }
  lastBalanceMicros = nowMicros;

  const float dtSeconds = static_cast<float>(elapsedMicros) * 0.000001f;
  const unsigned long nowMillis = millis();

  const SensorFrame& frame = sensors.update(nowMillis);
  const ControlCommand& command = bluetooth.update(nowMillis);

  if (command.hasTuning) {
    balance.setTunings(command.tuneKp, command.tuneKi, command.tuneKd);
    bluetooth.consumeTuning();
  }

  robotState.update(frame, command);

  MotorCommand motorOutput;
  if (robotState.motorsEnabled()) {
    const ControlCommand safe = robotState.safeCommand(command, frame);
    const float uprightAngle = robotState.uprightAngleDegrees();
    const float driveRatio =
        static_cast<float>(safe.forward) /
        static_cast<float>(Config::MaxDriveCommand);
    const float targetAngle =
        uprightAngle + driveRatio * Config::MaxTargetLeanDegrees;

    balance.setTargetAngle(targetAngle);
    const int16_t balanceOutput =
        balance.update(frame.angleDegrees, dtSeconds);
    motorOutput = mixer.mix(balanceOutput, 0, safe.turn);
    motors.write(motorOutput);
    lastCommand = safe;
  } else {
    balance.reset();
    motors.stop();
    lastCommand = command;
  }

  lastFrame = frame;
  lastMotorOutput = motorOutput;
  printDebug(lastFrame, lastCommand, lastMotorOutput);
}

void printMode(RobotMode mode) {
  switch (mode) {
  case RobotMode::Disarmed:
    Serial.print(F("disarmed"));
    break;
  case RobotMode::Calibrating:
    Serial.print(F("calibrating"));
    break;
  case RobotMode::Balancing:
    Serial.print(F("balancing"));
    break;
  case RobotMode::Drive:
    Serial.print(F("drive"));
    break;
  case RobotMode::Fault:
    Serial.print(F("fault"));
    break;
  }
}

void printDebug(const SensorFrame& frame, const ControlCommand& command,
                const MotorCommand& motorOutput) {
  if (!Config::EnableDebugSerial) {
    return;
  }

  if (frame.nowMillis - lastDebugMillis < Config::DebugPeriodMillis) {
    return;
  }
  lastDebugMillis = frame.nowMillis;

  Serial.print(F("mode="));
  printMode(robotState.mode());
  Serial.print(F(" angle="));
  Serial.print(frame.angleDegrees);
  Serial.print(F(" upright="));
  Serial.print(robotState.uprightAngleDegrees());
  Serial.print(F(" distance="));
  Serial.print(frame.distanceCm);
  Serial.print(F(" fwd="));
  Serial.print(command.forward);
  Serial.print(F(" turn="));
  Serial.print(command.turn);
  Serial.print(F(" left="));
  Serial.print(motorOutput.left);
  Serial.print(F(" right="));
  Serial.println(motorOutput.right);
}
