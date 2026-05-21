/*
  Hiwonder miniAuto control sketch.

  Hardware facts are from Hiwonder's miniAuto examples:
  - Motor PWM pins: D10, D9, D6, D11
  - Motor direction pins: D12, D8, D7, D13
  - Onboard WS2812 RGB data: D2
  - Passive buzzer: D3
  - Servo/gripper: D5
  - Battery divider: A3
  - Glowing ultrasonic sensor: I2C 0x77, distance in millimeters
  - 4-channel line sensor: I2C 0x78, line bits in register 1
*/

#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_ARCH_ZEPHYR)
#include <Arduino_RouterBridge.h>
#define HAS_ROUTER_BRIDGE 1
#define CMD_IO Monitor
#else
#define HAS_ROUTER_BRIDGE 0
#define CMD_IO Serial
#endif

#if defined(ARDUINO_ARCH_ZEPHYR)
const char MCU_NAME[] = "uno_q";
#else
const char MCU_NAME[] = "uno_r3";
#endif

const uint8_t PIN_RGB = 2;
const uint8_t PIN_BUZZER = 3;
const uint8_t PIN_SERVO = 5;
const uint8_t PIN_BATTERY = A3;

const uint8_t MOTOR_PWM_PIN[4] = {10, 9, 6, 11};
const uint8_t MOTOR_DIR_PIN[4] = {12, 8, 7, 13};
const uint8_t MOTOR_PWM_MIN = 2;
const int MAX_DRIVE_MS = 5000;
const int DEFAULT_PULSE_MS = 700;
const int DEFAULT_SPEED = 180;

const uint8_t ULTRASONIC_I2C_ADDR = 0x77;
const uint8_t LINE_FOLLOWER_I2C_ADDR = 0x78;
const uint8_t ULTRASONIC_RGB_MODE = 2;
const uint8_t ULTRASONIC_RGB1_R = 3;
const uint8_t ULTRASONIC_RGB_SIMPLE_MODE = 0;

unsigned long driveStopAt = 0;
bool driveTimerActive = false;
uint8_t speedPercent = 55;
bool obstacleAvoidEnabled = false;
unsigned long lastAvoidUpdate = 0;
String commandBuffer;
unsigned long lastCommandByteAt = 0;

int clampInt(int value, int low, int high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

uint8_t speedToPercent(int speed) {
  speed = abs(speed);
  speed = clampInt(speed, 0, 255);
  return (uint8_t)map(speed, 0, 255, 0, 100);
}

int clampDuration(int durationMs) {
  return clampInt(durationMs, 0, MAX_DRIVE_MS);
}

bool i2cWriteByte(uint8_t address, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cWriteData(uint8_t address, uint8_t reg, const uint8_t *values, uint8_t len) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(values[i]);
  }
  return Wire.endTransmission() == 0;
}

int i2cReadData(uint8_t address, uint8_t reg, uint8_t *values, uint8_t len) {
  if (!i2cWriteByte(address, reg)) {
    return -1;
  }

  uint8_t count = 0;
  Wire.requestFrom(address, len);
  while (Wire.available() && count < len) {
    values[count++] = Wire.read();
  }
  return count == len ? count : -1;
}

int readUltrasonicMm() {
  uint8_t bytes[2] = {0, 0};
  if (i2cReadData(ULTRASONIC_I2C_ADDR, 0, bytes, 2) != 2) {
    return -1;
  }
  return (int)(bytes[0] | (bytes[1] << 8));
}

void setUltrasonicColor(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t mode = ULTRASONIC_RGB_SIMPLE_MODE;
  uint8_t rgb[6] = {r, g, b, r, g, b};
  i2cWriteData(ULTRASONIC_I2C_ADDR, ULTRASONIC_RGB_MODE, &mode, 1);
  i2cWriteData(ULTRASONIC_I2C_ADDR, ULTRASONIC_RGB1_R, rgb, 6);
}

bool readLineBits(uint8_t bits[4]) {
  uint8_t data = 0;
  if (i2cReadData(LINE_FOLLOWER_I2C_ADDR, 1, &data, 1) != 1) {
    for (uint8_t i = 0; i < 4; i++) {
      bits[i] = 0;
    }
    return false;
  }

  bits[0] = data & 0x01;
  bits[1] = (data >> 1) & 0x01;
  bits[2] = (data >> 2) & 0x01;
  bits[3] = (data >> 3) & 0x01;
  return true;
}

int readBatteryMv() {
  return (int)(analogRead(PIN_BATTERY) * 29.89);
}

#if defined(ARDUINO_ARCH_AVR)
void rgbDelay(uint8_t cycles) {
  while (cycles--) {
    asm volatile("nop\n\t");
  }
}

void rgbSendBit(volatile uint8_t *port, uint8_t mask, bool one) {
  if (one) {
    *port |= mask;
    rgbDelay(11);
    *port &= ~mask;
    rgbDelay(5);
  } else {
    *port |= mask;
    rgbDelay(4);
    *port &= ~mask;
    rgbDelay(12);
  }
}

void rgbSendByte(volatile uint8_t *port, uint8_t mask, uint8_t value) {
  for (int8_t bit = 7; bit >= 0; bit--) {
    rgbSendBit(port, mask, value & (1 << bit));
  }
}

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  volatile uint8_t *port = portOutputRegister(digitalPinToPort(PIN_RGB));
  const uint8_t mask = digitalPinToBitMask(PIN_RGB);
  noInterrupts();
  rgbSendByte(port, mask, g);
  rgbSendByte(port, mask, r);
  rgbSendByte(port, mask, b);
  interrupts();
  delayMicroseconds(80);
}
#else
void rgbSendBitSlow(bool one) {
  digitalWrite(PIN_RGB, HIGH);
  if (one) {
    delayMicroseconds(1);
  }
  digitalWrite(PIN_RGB, LOW);
  if (!one) {
    delayMicroseconds(1);
  }
}

void rgbSendByteSlow(uint8_t value) {
  for (int8_t bit = 7; bit >= 0; bit--) {
    rgbSendBitSlow(value & (1 << bit));
  }
}

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  // Portable fallback for non-AVR cores. The I2C ultrasonic RGB is the reliable
  // status light on UNO Q; this best-effort WS2812 pulse path keeps builds portable.
  noInterrupts();
  rgbSendByteSlow(g);
  rgbSendByteSlow(r);
  rgbSendByteSlow(b);
  interrupts();
  delayMicroseconds(80);
}
#endif

void motorsSetPercent(int motor0, int motor1, int motor2, int motor3) {
  int motors[4] = {
    clampInt(motor0, -100, 100),
    clampInt(motor1, -100, 100),
    clampInt(motor2, -100, 100),
    clampInt(motor3, -100, 100)
  };
  bool baseDirection[4] = {true, false, false, true};

  for (uint8_t i = 0; i < 4; i++) {
    bool direction = baseDirection[i];
    if (motors[i] < 0) {
      direction = !direction;
    }

    uint8_t pwm = 0;
    if (motors[i] != 0) {
      pwm = (uint8_t)map(abs(motors[i]), 0, 100, MOTOR_PWM_MIN, 255);
    }

    digitalWrite(MOTOR_DIR_PIN[i], direction ? HIGH : LOW);
    analogWrite(MOTOR_PWM_PIN[i], pwm);
  }
}

void stopMotors() {
  motorsSetPercent(0, 0, 0, 0);
  driveTimerActive = false;
}

void velocityController(uint16_t angle, uint8_t velocity, int8_t rot, bool drift) {
  float speedFactor = (rot == 0) ? 1.0 : 0.5;
  float velocityScaled = velocity / sqrt(2.0);
  float rad = (angle + 90) * PI / 180.0;

  int motor0;
  int motor1;
  int motor2;
  int motor3;
  if (drift) {
    motor0 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor);
    motor1 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor);
    motor2 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor - rot * speedFactor * 2);
    motor3 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor + rot * speedFactor * 2);
  } else {
    motor0 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor + rot * speedFactor);
    motor1 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor - rot * speedFactor);
    motor2 = (int)((velocityScaled * sin(rad) - velocityScaled * cos(rad)) * speedFactor - rot * speedFactor);
    motor3 = (int)((velocityScaled * sin(rad) + velocityScaled * cos(rad)) * speedFactor + rot * speedFactor);
  }

  motorsSetPercent(motor0, motor1, motor2, motor3);
}

void armDriveTimer(int durationMs) {
  durationMs = clampDuration(durationMs);
  if (durationMs > 0) {
    driveStopAt = millis() + (unsigned long)durationMs;
    driveTimerActive = true;
  } else {
    driveTimerActive = false;
  }
}

void driveRaw(int motor0, int motor1, int motor2, int motor3, int durationMs) {
  motorsSetPercent(
    (int)map(clampInt(motor0, -255, 255), -255, 255, -100, 100),
    (int)map(clampInt(motor1, -255, 255), -255, 255, -100, 100),
    (int)map(clampInt(motor2, -255, 255), -255, 255, -100, 100),
    (int)map(clampInt(motor3, -255, 255), -255, 255, -100, 100)
  );
  armDriveTimer(durationMs);
}

bool driveCommand(String command, int speed, int durationMs) {
  command.trim();
  command.toLowerCase();
  const uint8_t percent = speedToPercent(speed);
  speedPercent = percent;

  if (command == "stop" || command == "x") {
    stopMotors();
    return true;
  }
  if (command == "forward" || command == "f") {
    velocityController(0, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "backward" || command == "back" || command == "reverse" || command == "b") {
    velocityController(180, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "left" || command == "strafe_left" || command == "a") {
    velocityController(90, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "right" || command == "strafe_right" || command == "d") {
    velocityController(270, percent, 0, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "rotate_left" || command == "turn_left" || command == "q") {
    velocityController(0, 0, percent, false);
    armDriveTimer(durationMs);
    return true;
  }
  if (command == "rotate_right" || command == "turn_right" || command == "e") {
    velocityController(0, 0, -percent, false);
    armDriveTimer(durationMs);
    return true;
  }

  return false;
}

void servoPulse(uint16_t pulseUs) {
  digitalWrite(PIN_SERVO, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(PIN_SERVO, LOW);
  delayMicroseconds(20000 - pulseUs);
}

bool setServoAngle(int angle) {
  angle = clampInt(angle, 0, 180);
  uint16_t pulseUs = (uint16_t)map(angle, 0, 180, 1000, 2000);
  for (uint8_t i = 0; i < 30; i++) {
    servoPulse(pulseUs);
  }
  return true;
}

void chirp() {
  for (uint8_t i = 0; i < 3; i++) {
    unsigned long endAt = millis() + 100;
    while (millis() < endAt) {
      digitalWrite(PIN_BUZZER, HIGH);
      delayMicroseconds(250);
      digitalWrite(PIN_BUZZER, LOW);
      delayMicroseconds(250);
    }
    delay(80);
  }
  digitalWrite(PIN_BUZZER, LOW);
}

String normalized(String input) {
  input.replace(',', ' ');
  input.replace('|', ' ');
  input.replace('(', ' ');
  input.replace(')', ' ');
  input.replace('&', ' ');
  input.trim();
  return input;
}

String tokenAt(String input, uint8_t wanted) {
  input = normalized(input);
  uint8_t index = 0;
  int start = 0;

  while (start < input.length()) {
    while (start < input.length() && input[start] == ' ') {
      start++;
    }
    int end = start;
    while (end < input.length() && input[end] != ' ') {
      end++;
    }
    if (end > start) {
      if (index == wanted) {
        return input.substring(start, end);
      }
      index++;
    }
    start = end + 1;
  }

  return "";
}

String readSensorsJson() {
  uint8_t lineBits[4];
  bool lineOk = readLineBits(lineBits);
  int distanceMm = readUltrasonicMm();
  int distanceCm = distanceMm >= 0 ? distanceMm / 10 : distanceMm;
  int batteryMv = readBatteryMv();

  String json = "{";
  json += "\"robot\":\"hiwonder_miniauto\"";
  json += ",\"mcu\":\"";
  json += MCU_NAME;
  json += "\"";
  json += ",\"ir\":-1";
  json += ",\"line_ok\":";
  json += lineOk ? "true" : "false";
  json += ",\"line_digital\":[";
  json += lineBits[0];
  json += ",";
  json += lineBits[1];
  json += ",";
  json += lineBits[2];
  json += ",";
  json += lineBits[3];
  json += "]";
  json += ",\"trace_digital\":[";
  json += lineBits[0];
  json += ",";
  json += lineBits[1];
  json += ",";
  json += lineBits[2];
  json += ",";
  json += lineBits[3];
  json += "]";
  json += ",\"ultrasonic_mm\":";
  json += distanceMm;
  json += ",\"ultrasonic_cm\":";
  json += distanceCm;
  json += ",\"battery_mv\":";
  json += batteryMv;
  json += "}";
  return json;
}

#if HAS_ROUTER_BRIDGE
bool rpcDrive(String command, int speed, int durationMs) {
  return driveCommand(command, speed, durationMs);
}

bool rpcStop() {
  stopMotors();
  obstacleAvoidEnabled = false;
  return true;
}

String rpcReadSensors() {
  return readSensorsJson();
}

bool rpcServo(int angle) {
  return setServoAngle(angle);
}

bool rpcBuzz() {
  chirp();
  return true;
}

bool rpcLed(bool on) {
  setRgb(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
  setUltrasonicColor(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
  return true;
}

bool rpcDriveRaw(int motor0, int motor1, int motor2, int motor3, int durationMs) {
  driveRaw(motor0, motor1, motor2, motor3, durationMs);
  return true;
}

String rpcHealth() {
  String json = "{\"robot\":\"hiwonder_miniauto\",\"mcu\":\"";
  json += MCU_NAME;
  json += "\",\"bridge\":true,\"serial\":true}";
  return json;
}

void registerBridgeMethods() {
  Bridge.provide_safe("drive", rpcDrive);
  Bridge.provide_safe("stop", rpcStop);
  Bridge.provide_safe("read_sensors", rpcReadSensors);
  Bridge.provide_safe("servo", rpcServo);
  Bridge.provide_safe("buzz", rpcBuzz);
  Bridge.provide_safe("led", rpcLed);
  Bridge.provide_safe("drive_raw", rpcDriveRaw);
  Bridge.provide_safe("health", rpcHealth);
}
#endif

void motorPulse(uint8_t motorIndex) {
  int motors[4] = {0, 0, 0, 0};
  motors[motorIndex] = 100;
  motorsSetPercent(motors[0], motors[1], motors[2], motors[3]);
  delay(550);
  motors[motorIndex] = -100;
  motorsSetPercent(motors[0], motors[1], motors[2], motors[3]);
  delay(550);
  stopMotors();
}

void printHelp() {
  CMD_IO.println();
  CMD_IO.println(F("Hiwonder miniAuto commands:"));
  CMD_IO.println(F("  ? help"));
  CMD_IO.println(F("  r read sensors"));
  CMD_IO.println(F("  l RGB blink"));
  CMD_IO.println(F("  z buzzer chirp"));
  CMD_IO.println(F("  u ultrasonic distance"));
  CMD_IO.println(F("  v servo center-open-close-center"));
  CMD_IO.println(F("  1..4 motor tests: 1=front-left, 2=front-right, 3=rear-right, 4=rear-left"));
  CMD_IO.println(F("  f/b/a/d/q/e/x forward/back/left/right/rotate-left/rotate-right/stop"));
  CMD_IO.println(F("Line API: drive(command,speed,ms), stop, read_sensors, servo(angle), buzz, led(on), rgb(r,g,b), drive_raw(m0,m1,m2,m3,ms), health"));
  CMD_IO.println(F("Hiwonder protocol also works: A|2|&, B|255|0|0|&, C|50|&, D|&, E|30|&, F|0|&"));
  CMD_IO.println();
}

void handleSingleCommand(char command) {
  switch (command) {
    case '?':
      printHelp();
      break;
    case 'r':
      CMD_IO.println(readSensorsJson());
      break;
    case 'l':
      setRgb(255, 0, 0);
      setUltrasonicColor(255, 0, 0);
      delay(180);
      setRgb(0, 255, 0);
      setUltrasonicColor(0, 255, 0);
      delay(180);
      setRgb(0, 0, 255);
      setUltrasonicColor(0, 0, 255);
      delay(180);
      setRgb(0, 0, 0);
      setUltrasonicColor(0, 0, 0);
      CMD_IO.println(F("OK led"));
      break;
    case 'z':
      chirp();
      CMD_IO.println(F("OK buzz"));
      break;
    case 'u':
      CMD_IO.print(F("ultrasonic_mm="));
      CMD_IO.println(readUltrasonicMm());
      break;
    case 'v':
      setServoAngle(90);
      setServoAngle(150);
      setServoAngle(30);
      setServoAngle(90);
      CMD_IO.println(F("OK servo"));
      break;
    case '1':
      motorPulse(0);
      CMD_IO.println(F("OK M0 front-left"));
      break;
    case '2':
      motorPulse(1);
      CMD_IO.println(F("OK M1 front-right"));
      break;
    case '3':
      motorPulse(2);
      CMD_IO.println(F("OK M2 rear-right"));
      break;
    case '4':
      motorPulse(3);
      CMD_IO.println(F("OK M3 rear-left"));
      break;
    case 'f':
      driveCommand("forward", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK forward"));
      break;
    case 'b':
      driveCommand("backward", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK backward"));
      break;
    case 'a':
      driveCommand("left", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK left"));
      break;
    case 'd':
      driveCommand("right", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK right"));
      break;
    case 'q':
      driveCommand("rotate_left", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK rotate_left"));
      break;
    case 'e':
      driveCommand("rotate_right", DEFAULT_SPEED, DEFAULT_PULSE_MS);
      CMD_IO.println(F("OK rotate_right"));
      break;
    case 'x':
      stopMotors();
      obstacleAvoidEnabled = false;
      CMD_IO.println(F("OK stop"));
      break;
    default:
      CMD_IO.print(F("ERR unknown command "));
      CMD_IO.println(command);
      break;
  }
}

void handleHiwonderProtocol(String line) {
  String function = tokenAt(line, 0);
  function.toUpperCase();

  if (function == "A") {
    uint8_t state = (uint8_t)tokenAt(line, 1).toInt();
    switch (state) {
      case 0: velocityController(90, speedPercent, 0, false); break;
      case 1: velocityController(45, speedPercent, 0, false); break;
      case 2: velocityController(0, speedPercent, 0, false); break;
      case 3: velocityController(315, speedPercent, 0, false); break;
      case 4: velocityController(270, speedPercent, 0, false); break;
      case 5: velocityController(225, speedPercent, 0, false); break;
      case 6: velocityController(180, speedPercent, 0, false); break;
      case 7: velocityController(135, speedPercent, 0, false); break;
      case 8: stopMotors(); break;
      case 9: velocityController(0, 0, speedPercent, false); break;
      case 10: velocityController(0, 0, -speedPercent, false); break;
      case 11: stopMotors(); break;
      default: stopMotors(); break;
    }
    CMD_IO.println(F("A|OK|$"));
    return;
  }

  if (function == "B") {
    uint8_t r = (uint8_t)clampInt(tokenAt(line, 1).toInt(), 0, 255);
    uint8_t g = (uint8_t)clampInt(tokenAt(line, 2).toInt(), 0, 255);
    uint8_t b = (uint8_t)clampInt(tokenAt(line, 3).toInt(), 0, 255);
    setRgb(r, g, b);
    setUltrasonicColor(r, g, b);
    CMD_IO.println(F("B|OK|$"));
    return;
  }

  if (function == "C") {
    speedPercent = (uint8_t)clampInt(tokenAt(line, 1).toInt(), 10, 100);
    CMD_IO.print(F("C|"));
    CMD_IO.print(speedPercent);
    CMD_IO.println(F("|$"));
    return;
  }

  if (function == "D") {
    CMD_IO.print(F("$"));
    CMD_IO.print(readUltrasonicMm());
    CMD_IO.print(F(","));
    CMD_IO.print(readBatteryMv());
    CMD_IO.println(F("$"));
    return;
  }

  if (function == "E") {
    int increase = clampInt(tokenAt(line, 1).toInt(), 0, 60);
    setServoAngle(90 + increase);
    CMD_IO.println(F("E|OK|$"));
    return;
  }

  if (function == "F") {
    obstacleAvoidEnabled = tokenAt(line, 1).toInt() != 0;
    if (!obstacleAvoidEnabled) {
      stopMotors();
    }
    CMD_IO.println(F("F|OK|$"));
    return;
  }

  CMD_IO.println(F("ERR hiwonder protocol"));
}

void handleLineCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.endsWith("&")) {
    handleHiwonderProtocol(line);
    return;
  }

  if (line.length() == 1) {
    handleSingleCommand(line[0]);
    return;
  }

  String command = tokenAt(line, 0);
  command.toLowerCase();

  if (command == "drive") {
    String direction = tokenAt(line, 1);
    int speed = tokenAt(line, 2).length() ? tokenAt(line, 2).toInt() : DEFAULT_SPEED;
    int duration = tokenAt(line, 3).length() ? tokenAt(line, 3).toInt() : 0;
    CMD_IO.println(driveCommand(direction, speed, duration) ? F("OK drive") : F("ERR drive"));
    return;
  }

  if (command == "stop") {
    stopMotors();
    obstacleAvoidEnabled = false;
    CMD_IO.println(F("OK stop"));
    return;
  }

  if (command == "read_sensors" || command == "sensors") {
    CMD_IO.println(readSensorsJson());
    return;
  }

  if (command == "servo") {
    setServoAngle(tokenAt(line, 1).toInt());
    CMD_IO.println(F("OK servo"));
    return;
  }

  if (command == "buzz") {
    chirp();
    CMD_IO.println(F("OK buzz"));
    return;
  }

  if (command == "led") {
    bool on = tokenAt(line, 1).toInt() != 0;
    setRgb(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
    setUltrasonicColor(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
    CMD_IO.println(F("OK led"));
    return;
  }

  if (command == "rgb") {
    uint8_t r = (uint8_t)clampInt(tokenAt(line, 1).toInt(), 0, 255);
    uint8_t g = (uint8_t)clampInt(tokenAt(line, 2).toInt(), 0, 255);
    uint8_t b = (uint8_t)clampInt(tokenAt(line, 3).toInt(), 0, 255);
    setRgb(r, g, b);
    setUltrasonicColor(r, g, b);
    CMD_IO.println(F("OK rgb"));
    return;
  }

  if (command == "drive_raw") {
    int m0 = tokenAt(line, 1).toInt();
    int m1 = tokenAt(line, 2).toInt();
    int m2 = tokenAt(line, 3).toInt();
    int m3 = tokenAt(line, 4).toInt();
    int duration = tokenAt(line, 5).length() ? tokenAt(line, 5).toInt() : 0;
    driveRaw(m0, m1, m2, m3, duration);
    CMD_IO.println(F("OK drive_raw"));
    return;
  }

  if (command == "health") {
    String json = "{\"robot\":\"hiwonder_miniauto\",\"mcu\":\"";
    json += MCU_NAME;
    json += "\",\"serial\":true";
#if HAS_ROUTER_BRIDGE
    json += ",\"bridge\":true";
#endif
    json += "}";
    CMD_IO.println(json);
    return;
  }

  CMD_IO.println(F("ERR unknown line command"));
}

bool isSingleCommandChar(char command) {
  switch (command) {
    case '?':
    case 'r':
    case 'l':
    case 'z':
    case 'u':
    case 'v':
    case '1':
    case '2':
    case '3':
    case '4':
    case 'f':
    case 'b':
    case 'a':
    case 'd':
    case 'q':
    case 'e':
    case 'x':
      return true;
    default:
      return false;
  }
}

void pollSerial() {
  bool received = false;

  while (CMD_IO.available()) {
    char incoming = (char)CMD_IO.read();
    received = true;
    lastCommandByteAt = millis();

    if (incoming == '\r' || incoming == '\n') {
      if (commandBuffer.length() > 0) {
        handleLineCommand(commandBuffer);
        commandBuffer = "";
      }
      continue;
    }

    if (commandBuffer.length() == 0 && isSingleCommandChar(incoming) && CMD_IO.available() == 0) {
      handleSingleCommand(incoming);
      continue;
    }

    commandBuffer += incoming;
    if (incoming == '&' || incoming == ')' || commandBuffer.length() >= 96) {
      handleLineCommand(commandBuffer);
      commandBuffer = "";
    }
  }

  if (!received && commandBuffer.length() > 0 && millis() - lastCommandByteAt > 80) {
    handleLineCommand(commandBuffer);
    commandBuffer = "";
  }
}

void updateDriveTimer() {
  if (driveTimerActive && (long)(millis() - driveStopAt) >= 0) {
    stopMotors();
  }
}

void updateObstacleAvoid() {
  if (!obstacleAvoidEnabled || millis() - lastAvoidUpdate < 100) {
    return;
  }

  lastAvoidUpdate = millis();
  int distanceMm = readUltrasonicMm();
  if (distanceMm > 0 && distanceMm < 400) {
    velocityController(0, 0, speedPercent, false);
  } else {
    velocityController(0, speedPercent, 0, false);
  }
}

void setupPins() {
  pinMode(PIN_RGB, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_PWM_PIN[i], OUTPUT);
    pinMode(MOTOR_DIR_PIN[i], OUTPUT);
  }

  digitalWrite(PIN_BUZZER, LOW);
  stopMotors();
  setRgb(0, 0, 0);
}

void setup() {
  CMD_IO.begin(9600);
  CMD_IO.setTimeout(80);
  Wire.begin();
  setupPins();
  setUltrasonicColor(0, 0, 0);
#if HAS_ROUTER_BRIDGE
  if (Bridge.begin()) {
    registerBridgeMethods();
  }
#endif
  CMD_IO.println(F("Hiwonder miniAuto command sketch ready."));
  printHelp();
}

void loop() {
  pollSerial();
  updateDriveTimer();
  updateObstacleAvoid();
}
