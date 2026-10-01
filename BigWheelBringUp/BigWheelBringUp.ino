/*
 * BigWheelBringUp — one-shot hardware check for the big-wheel build, run
 * BEFORE BigWheelBalance.  Open the serial monitor at 115200 (opening it resets
 * the board and starts the sequence).
 *
 * 1. MOTOR PORTS.  Lay the robot down or hold it with the wheels OFF the
 *    ground.  After 3 s, each MegaPi DC port (PORT1A..PORT4A, then
 *    PORT1B..PORT4B) is driven at +kTestPwm for 1.5 s, one at a time.  Before
 *    each step the LED blinks the step number.  Note which wheel turns on
 *    which step, and which way the robot would roll for that +PWM.
 * 2. IMU AXES.  Then it streams the MPU6050 at 10 Hz (accel in g, gyro in
 *    deg/s, I2C error count).  Stand the robot upright, then lean it FORWARD
 *    (the way it should roll for a positive command), hold, then lean it back.
 *
 * Same I2C clock / sensor setup as BigWheelBalance, so a clean stream here
 * also validates the bus.  Press reset to run it again.  Opening the serial
 * port resets the board; to stream the IMU without re-running the motors,
 * build with --build-property "compiler.cpp.extra_flags=-DBRINGUP_IMU_ONLY".
 *
 * -DBRINGUP_BT (no motors): sniff the Makeblock Bluetooth controller on Serial3
 * (MePS2 packet format: FF 55, LX, btn, LY, btn, RX, btn, RY, checksum) and
 * print the decoded sticks/buttons, packet rate, raw byte count and the yaw
 * gyro (Z) at 10 Hz, so the stick mapping and the turn sign can be checked.
 */

#include <MeMegaPi.h>  // also declares Wire (Makeblock's old utility/Wire.h)

// See BigWheelBalance: the core's TWI timeout, reachable past Makeblock's header.
extern "C" void twi_setTimeoutInMicros(uint32_t timeout, bool resetWithTimeout);

static const int16_t kTestPwm = 120;        // ~5.6 V on a 12 V pack
static const uint32_t kStepOnMs = 1500UL;
static const uint32_t kStepOffMs = 1000UL;
static const uint8_t kImuAddress = 0x68;
static const uint32_t kI2cClockHz = 400000UL;  // keep equal to BigWheelBalance

struct PortName {
  uint8_t port;
  const char* name;
};
static const PortName kPorts[] = {
    {PORT1A, "PORT1A"}, {PORT2A, "PORT2A"}, {PORT3A, "PORT3A"}, {PORT4A, "PORT4A"},
    {PORT1B, "PORT1B"}, {PORT2B, "PORT2B"}, {PORT3B, "PORT3B"}, {PORT4B, "PORT4B"},
};

#ifdef BRINGUP_BT
#define BRINGUP_IMU_ONLY
#endif

static MeMegaPiDCMotor gMotor;
static uint32_t gI2cErrors = 0;

#ifdef BRINGUP_BT
// MePS2-format parser: after FF 55 come 8 bytes, the last being the 8-bit sum
// of the first seven.
static uint8_t gPkt[8];
static uint8_t gPktIndex = 0;
static uint8_t gPktState = 0;  // 0 = want FF, 1 = want 55, 2 = payload
static uint8_t gLast[8] = {0x80, 0, 0x80, 0, 0x80, 0, 0x80, 0};
static uint16_t gPktCount = 0;
static uint16_t gCksErrors = 0;
static uint32_t gRawBytes = 0;
static uint8_t gFirstRaw[32];
static uint8_t gFirstRawCount = 0;

static void pollController() {
  while (Serial3.available() > 0) {
    const uint8_t c = static_cast<uint8_t>(Serial3.read());
    ++gRawBytes;
    if (gFirstRawCount < sizeof(gFirstRaw)) {
      gFirstRaw[gFirstRawCount++] = c;
    }
    if (gPktState == 2) {
      gPkt[gPktIndex++] = c;
      if (gPktIndex == 8) {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 7; ++i) {
          sum = static_cast<uint8_t>(sum + gPkt[i]);
        }
        if (sum == gPkt[7]) {
          memcpy(gLast, gPkt, sizeof(gLast));
          ++gPktCount;
        } else {
          ++gCksErrors;
        }
        gPktState = 0;
      }
    } else if (c == 0xFF) {
      gPktState = 1;
    } else if (gPktState == 1 && c == 0x55) {
      gPktState = 2;
      gPktIndex = 0;
    } else {
      gPktState = 0;
    }
  }
}
#endif

static void configureDcMotorPwmTimers() {  // same as BigWheelBalance
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);
}

#ifndef BRINGUP_IMU_ONLY
static void blink(uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}
#endif

static void writeImuReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static bool readImu(int16_t raw[7]) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(static_cast<uint8_t>(0x3B));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(kImuAddress, static_cast<uint8_t>(14)) != 14) {
    return false;
  }
  for (uint8_t i = 0; i < 7; ++i) {
    const uint8_t hi = static_cast<uint8_t>(Wire.read());
    const uint8_t lo = static_cast<uint8_t>(Wire.read());
    raw[i] = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  }
  return true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  configureDcMotorPwmTimers();

  Wire.begin();
  Wire.setClock(kI2cClockHz);
  twi_setTimeoutInMicros(3000, true);
  Wire.beginTransmission(kImuAddress);
  Wire.write(static_cast<uint8_t>(0x75));  // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom(kImuAddress, static_cast<uint8_t>(1));
  const int who = Wire.available() ? Wire.read() : -1;
  writeImuReg(0x6B, 0x01);
  delay(50);
  writeImuReg(0x19, 0x00);
  writeImuReg(0x1A, 0x02);
  writeImuReg(0x1B, 0x08);
  writeImuReg(0x1C, 0x08);

  Serial.println(F("BigWheelBringUp"));
#ifdef BRINGUP_BT
  Serial3.begin(115200);
#endif
  Serial.print(F("IMU WHO_AM_I = 0x"));
  Serial.println(who, HEX);
#ifndef BRINGUP_IMU_ONLY
  Serial.println(F("MOTOR TEST in 3 s - wheels OFF the ground"));
  delay(3000);

  for (uint8_t i = 0; i < sizeof(kPorts) / sizeof(kPorts[0]); ++i) {
    blink(static_cast<uint8_t>(i + 1));
    Serial.print(F("STEP "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(kPorts[i].name);
    Serial.print(F(" pwm=+"));
    Serial.println(kTestPwm);
    gMotor.reset(kPorts[i].port);
    gMotor.run(kTestPwm);
    delay(kStepOnMs);
    gMotor.run(0);
    delay(kStepOffMs);
  }
  Serial.println(F("MOTOR TEST DONE"));
#endif
#ifdef BRINGUP_BT
  Serial.println(F("BT STREAM: LX LY RX RY  btnA btnB btnC  pkts/0.1s cksErr rawBytes  gz[deg/s]"));
#else
  Serial.println(F("IMU STREAM: ax ay az [g]  gx gy gz [deg/s]  i2cErr"));
#endif
}

void loop() {
  static uint32_t lastMs = 0;
#ifdef BRINGUP_BT
  pollController();
#endif
  const uint32_t now = millis();
  if (now - lastMs < 100UL) {
    return;
  }
  lastMs = now;
  int16_t raw[7];
  if (!readImu(raw)) {
    ++gI2cErrors;
    return;
  }
#ifdef BRINGUP_BT
  static bool dumped = false;
  if (!dumped && gFirstRawCount == sizeof(gFirstRaw)) {
    dumped = true;
    Serial.print(F("RAW:"));
    for (uint8_t i = 0; i < gFirstRawCount; ++i) {
      Serial.print(' ');
      Serial.print(gFirstRaw[i], HEX);
    }
    Serial.println();
  }
  Serial.print(gLast[0]); Serial.print(' ');
  Serial.print(gLast[2]); Serial.print(' ');
  Serial.print(gLast[4]); Serial.print(' ');
  Serial.print(gLast[6]); Serial.print(F("  "));
  Serial.print(gLast[1], HEX); Serial.print(' ');
  Serial.print(gLast[3], HEX); Serial.print(' ');
  Serial.print(gLast[5], HEX); Serial.print(F("  "));
  Serial.print(gPktCount); Serial.print(' ');
  Serial.print(gCksErrors); Serial.print(' ');
  Serial.print(gRawBytes); Serial.print(F("  "));
  Serial.println(static_cast<float>(raw[6]) / 65.5f, 1);
  gPktCount = 0;
  return;
#endif
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print(static_cast<float>(raw[i]) / 8192.0f, 3);
    Serial.print(' ');
  }
  for (uint8_t i = 4; i < 7; ++i) {
    Serial.print(static_cast<float>(raw[i]) / 65.5f, 1);
    Serial.print(' ');
  }
  Serial.println(gI2cErrors);
}
