/*
  ACEBOTT ESP32 Car Shield v2.0 control app for Arduino UNO Q.

  The MCU side owns deterministic hardware control. The Linux/Qualcomm side can
  call Bridge RPC methods such as drive, stop, read_sensors, servo, and buzz.

  IMPORTANT:
  UNO Q GPIO is 3.3 V. Do not connect 5 V sensor outputs directly to UNO Q
  inputs. Use level shifters or resistor dividers on ECHO, trace, IR, etc.
*/

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

const int8_t PIN_NOT_AVAILABLE = -1;

// Outputs from UNO Q to shield/modules. These are physical UNO-style header
// positions corresponding to the ESP32 GPIO labels on the ACEBOTT ESP32 Max.
const int8_t PIN_LED_LEFT = 7;                  // ACEBOTT GPIO12
const int8_t PIN_LED_RIGHT = PIN_NOT_AVAILABLE; // ACEBOTT GPIO2 is on an extra ESP32-Max pin
const int8_t PIN_BUZZER = A1;                   // ACEBOTT GPIO33
const int8_t PIN_SERVO = 4;                     // ACEBOTT GPIO25
const int8_t PIN_US_TRIG = 6;                   // ACEBOTT GPIO13

// Inputs to UNO Q. These must be level-shifted to 3.3 V max.
const int8_t PIN_IR = PIN_NOT_AVAILABLE;        // ACEBOTT GPIO4 is on an extra ESP32-Max pin
const int8_t PIN_US_ECHO = 5;                   // ACEBOTT GPIO14
const int8_t PIN_TRACE_L = A3;                  // ACEBOTT GPIO35
const int8_t PIN_TRACE_M = A4;                  // ACEBOTT GPIO36
const int8_t PIN_TRACE_R = A5;                  // ACEBOTT GPIO39

// ACEBOTT V2 motor controller UART TX candidates. The regular motor commands
// broadcast to these pins because the UNO Q header naming does not line up with
// ACEBOTT's ESP32-Max labels, and the sweep proved one reaches the controller.
const int8_t V2_MOTOR_TX_CANDIDATES[] = {10, 11, 8, 9, 12, 13};
const uint16_t V2_UART_BIT_US = 104; // 9600 baud, 8N1
const int MOTOR_TEST_SPEED = 255;
const uint16_t MOTOR_PULSE_MS = 700;
const int MAX_DRIVE_MS = 5000;

// Observed on this UNO Q + ACEBOTT shield:
// M1 = left front, M2 = left back, M3 = right front, M4 = right back.
enum MotorIndex : uint8_t {
  MOTOR_LEFT_FRONT = 1,
  MOTOR_LEFT_BACK = 2,
  MOTOR_RIGHT_FRONT = 3,
  MOTOR_RIGHT_BACK = 4,
};

bool hasPin(int8_t pin) {
  return pin >= 0;
}

void pinModeIfAvailable(int8_t pin, PinMode mode) {
  if (hasPin(pin)) {
    pinMode((uint8_t)pin, mode);
  }
}

void digitalWriteIfAvailable(int8_t pin, PinStatus value) {
  if (hasPin(pin)) {
    digitalWrite((uint8_t)pin, value);
  }
}

int digitalReadIfAvailable(int8_t pin) {
  return hasPin(pin) ? digitalRead((uint8_t)pin) : -1;
}

int analogReadIfAvailable(int8_t pin) {
  return hasPin(pin) ? analogRead((uint8_t)pin) : -1;
}

int clampSpeed(int speed) {
  if (speed < 0) {
    speed = -speed;
  }
  if (speed > 255) {
    return 255;
  }
  return speed;
}

int clampSignedSpeed(int speed) {
  if (speed > 255) {
    return 255;
  }
  if (speed < -255) {
    return -255;
  }
  return speed;
}

int clampDuration(int durationMs) {
  if (durationMs < 0) {
    return 0;
  }
  if (durationMs > MAX_DRIVE_MS) {
    return MAX_DRIVE_MS;
  }
  return durationMs;
}

void boardLedWrite(bool on) {
#if defined(LED3_R)
  // UNO Q RGB LEDs are active-low on the current Zephyr core.
  digitalWrite(LED3_R, on ? LOW : HIGH);
#elif defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

void v2MotorTxByte(int8_t txPin, uint8_t value) {
  if (!hasPin(txPin)) {
    return;
  }

  noInterrupts();
  digitalWrite((uint8_t)txPin, LOW);
  delayMicroseconds(V2_UART_BIT_US);

  for (uint8_t bit = 0; bit < 8; bit++) {
    digitalWrite((uint8_t)txPin, (value & (1 << bit)) ? HIGH : LOW);
    delayMicroseconds(V2_UART_BIT_US);
  }

  digitalWrite((uint8_t)txPin, HIGH);
  delayMicroseconds(V2_UART_BIT_US);
  interrupts();
}

void v2MotorControlOnPin(int8_t txPin, uint8_t motor, int speed) {
  if (!hasPin(txPin) || motor < 1 || motor > 4) {
    return;
  }

  speed = clampSignedSpeed(speed);

  const uint8_t direction = (motor == 1 || motor == 2)
    ? (speed > 0 ? 1 : 2)
    : (speed > 0 ? 2 : 1);
  uint8_t speedValue = (uint8_t)((abs(speed) * 100L) / 255L);
  if (speedValue == 40) {
    speedValue = 0;
  }

  pinModeIfAvailable(txPin, OUTPUT);
  digitalWriteIfAvailable(txPin, HIGH);
  delayMicroseconds(V2_UART_BIT_US * 2);
  v2MotorTxByte(txPin, motor);
  v2MotorTxByte(txPin, direction);
  v2MotorTxByte(txPin, speedValue);
  v2MotorTxByte(txPin, '\r');
  v2MotorTxByte(txPin, '\n');
  delay(4);
}

void v2MoveOnPin(int8_t txPin, int m1, int m2, int m3, int m4) {
  v2MotorControlOnPin(txPin, MOTOR_LEFT_FRONT, m1);
  v2MotorControlOnPin(txPin, MOTOR_LEFT_BACK, m2);
  v2MotorControlOnPin(txPin, MOTOR_RIGHT_FRONT, m3);
  v2MotorControlOnPin(txPin, MOTOR_RIGHT_BACK, m4);
}

void v2StopOnPin(int8_t txPin) {
  v2MoveOnPin(txPin, 0, 0, 0, 0);
}

void v2MotorControlAllCandidates(uint8_t motor, int speed) {
  for (uint8_t i = 0; i < sizeof(V2_MOTOR_TX_CANDIDATES) / sizeof(V2_MOTOR_TX_CANDIDATES[0]); i++) {
    v2MotorControlOnPin(V2_MOTOR_TX_CANDIDATES[i], motor, speed);
  }
}

void v2MoveAllCandidates(int m1, int m2, int m3, int m4) {
  for (uint8_t i = 0; i < sizeof(V2_MOTOR_TX_CANDIDATES) / sizeof(V2_MOTOR_TX_CANDIDATES[0]); i++) {
    v2MoveOnPin(V2_MOTOR_TX_CANDIDATES[i], m1, m2, m3, m4);
  }
}

void v2StopAllCandidates() {
  for (uint8_t i = 0; i < sizeof(V2_MOTOR_TX_CANDIDATES) / sizeof(V2_MOTOR_TX_CANDIDATES[0]); i++) {
    v2StopOnPin(V2_MOTOR_TX_CANDIDATES[i]);
  }
}

void driveRaw(int m1, int m2, int m3, int m4, int durationMs) {
  v2MoveAllCandidates(
    clampSignedSpeed(m1),
    clampSignedSpeed(m2),
    clampSignedSpeed(m3),
    clampSignedSpeed(m4)
  );

  durationMs = clampDuration(durationMs);
  if (durationMs > 0) {
    delay(durationMs);
    v2StopAllCandidates();
  }
}

bool driveCommand(String command, int speed, int durationMs) {
  command.trim();
  command.toLowerCase();
  speed = clampSpeed(speed);

  if (command == "stop" || command == "x") {
    v2StopAllCandidates();
    return true;
  }
  if (command == "forward" || command == "f") {
    driveRaw(speed, speed, speed, speed, durationMs);
    return true;
  }
  if (command == "backward" || command == "back" || command == "reverse" || command == "b") {
    driveRaw(-speed, -speed, -speed, -speed, durationMs);
    return true;
  }
  if (command == "left" || command == "strafe_left" || command == "a") {
    driveRaw(-speed, speed, speed, -speed, durationMs);
    return true;
  }
  if (command == "right" || command == "strafe_right" || command == "d") {
    driveRaw(speed, -speed, -speed, speed, durationMs);
    return true;
  }
  if (command == "rotate_left" || command == "turn_left" || command == "q") {
    driveRaw(-speed, -speed, speed, speed, durationMs);
    return true;
  }
  if (command == "rotate_right" || command == "turn_right" || command == "e") {
    driveRaw(speed, speed, -speed, -speed, durationMs);
    return true;
  }

  return false;
}

long readUltrasonicCm() {
  if (!hasPin(PIN_US_TRIG) || !hasPin(PIN_US_ECHO)) {
    return -2;
  }
  digitalWriteIfAvailable(PIN_US_TRIG, LOW);
  delayMicroseconds(3);
  digitalWriteIfAvailable(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWriteIfAvailable(PIN_US_TRIG, LOW);

  const unsigned long duration = pulseIn((uint8_t)PIN_US_ECHO, HIGH, 25000UL);
  if (duration == 0) {
    return -1;
  }
  return (long)(duration / 58UL);
}

String readSensorsJson() {
  String json = "{";
  json += "\"ir\":";
  json += digitalReadIfAvailable(PIN_IR);
  json += ",\"trace_digital\":[";
  json += digitalReadIfAvailable(PIN_TRACE_L);
  json += ",";
  json += digitalReadIfAvailable(PIN_TRACE_M);
  json += ",";
  json += digitalReadIfAvailable(PIN_TRACE_R);
  json += "],\"trace_analog\":[";
  json += analogReadIfAvailable(PIN_TRACE_L);
  json += ",";
  json += analogReadIfAvailable(PIN_TRACE_M);
  json += ",";
  json += analogReadIfAvailable(PIN_TRACE_R);
  json += "],\"ultrasonic_cm\":";
  json += readUltrasonicCm();
  json += "}";
  return json;
}

void servoPulse(uint16_t pulseUs) {
  if (!hasPin(PIN_SERVO)) {
    return;
  }
  digitalWriteIfAvailable(PIN_SERVO, HIGH);
  delayMicroseconds(pulseUs);
  digitalWriteIfAvailable(PIN_SERVO, LOW);
  delayMicroseconds(20000 - pulseUs);
}

void holdServo(uint16_t pulseUs, uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    servoPulse(pulseUs);
  }
}

bool setServoAngle(int angle) {
  if (!hasPin(PIN_SERVO)) {
    return false;
  }
  if (angle < 0) {
    angle = 0;
  } else if (angle > 180) {
    angle = 180;
  }

  const uint16_t pulseUs = (uint16_t)map(angle, 0, 180, 1000, 2000);
  holdServo(pulseUs, 30);
  return true;
}

void chirp() {
  for (uint8_t i = 0; i < 3; i++) {
    const unsigned long endAt = millis() + 120;
    while (millis() < endAt) {
      digitalWriteIfAvailable(PIN_BUZZER, HIGH);
      delayMicroseconds(250);
      digitalWriteIfAvailable(PIN_BUZZER, LOW);
      delayMicroseconds(250);
    }
    delay(90);
  }
  digitalWriteIfAvailable(PIN_BUZZER, LOW);
}

void blinkLeds() {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWriteIfAvailable(PIN_LED_LEFT, LOW);
    digitalWriteIfAvailable(PIN_LED_RIGHT, HIGH);
    delay(150);
    digitalWriteIfAvailable(PIN_LED_LEFT, HIGH);
    digitalWriteIfAvailable(PIN_LED_RIGHT, LOW);
    delay(150);
  }
  digitalWriteIfAvailable(PIN_LED_LEFT, HIGH);
  digitalWriteIfAvailable(PIN_LED_RIGHT, HIGH);
}

void motorPulse(uint8_t motor) {
  v2MotorControlAllCandidates(motor, MOTOR_TEST_SPEED);
  delay(650);
  v2MotorControlAllCandidates(motor, 0);
  delay(250);
  v2MotorControlAllCandidates(motor, -MOTOR_TEST_SPEED);
  delay(650);
  v2MotorControlAllCandidates(motor, 0);
}

void motorSweep() {
  Monitor.println("Lift wheels. Trying V2 motor-controller TX candidate pins...");
  for (uint8_t i = 0; i < sizeof(V2_MOTOR_TX_CANDIDATES) / sizeof(V2_MOTOR_TX_CANDIDATES[0]); i++) {
    const int8_t txPin = V2_MOTOR_TX_CANDIDATES[i];
    Monitor.print("Trying V2 TX on D");
    Monitor.println(txPin);
    Monitor.flush();

    v2MoveOnPin(txPin, 255, 255, 255, 255);
    delay(700);
    v2StopOnPin(txPin);
    delay(300);
    v2MoveOnPin(txPin, -255, -255, -255, -255);
    delay(700);
    v2StopOnPin(txPin);
    delay(700);
  }
  v2StopAllCandidates();
  Monitor.println("V2 TX sweep done.");
}

bool rpcDrive(String command, int speed, int durationMs) {
  return driveCommand(command, speed, durationMs);
}

bool rpcStop() {
  v2StopAllCandidates();
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
  digitalWriteIfAvailable(PIN_LED_LEFT, on ? LOW : HIGH);
  return true;
}

bool rpcDriveRaw(int m1, int m2, int m3, int m4, int durationMs) {
  driveRaw(m1, m2, m3, m4, durationMs);
  return true;
}

String rpcHealth() {
  return String("{\"robot\":\"acebott_robocar\",\"mcu\":\"uno_q\",\"bridge\":true}");
}

void printHelp() {
  Monitor.println();
  Monitor.println("ACEBOTT UNO Q car commands:");
  Monitor.println("  ?  help");
  Monitor.println("  r  read sensors");
  Monitor.println("  l  blink LEDs");
  Monitor.println("  z  short buzzer chirp");
  Monitor.println("  u  read ultrasonic distance");
  Monitor.println("  v  servo center-left-right-center");
  Monitor.println("  1  test M1 left-front forward/reverse");
  Monitor.println("  2  test M2 left-back forward/reverse");
  Monitor.println("  3  test M3 right-front forward/reverse");
  Monitor.println("  4  test M4 right-back forward/reverse");
  Monitor.println("  f  all motors forward for 700 ms");
  Monitor.println("  b  all motors reverse for 700 ms");
  Monitor.println("  a  strafe left for 700 ms");
  Monitor.println("  d  strafe right for 700 ms");
  Monitor.println("  q  rotate left for 700 ms");
  Monitor.println("  e  rotate right for 700 ms");
  Monitor.println("  m  sweep V2 motor UART TX candidates D10,D11,D8,D9,D12,D13");
  Monitor.println("  x  stop motors");
  Monitor.println("Bridge RPC: drive(command,speed,ms), stop(), read_sensors(), servo(angle), buzz()");
  Monitor.println("Right LED and IR are unavailable without access to ESP32-Max extra pins.");
  Monitor.println();
}

void handleMonitorCommand(char command) {
  switch (command) {
    case '?':
      printHelp();
      break;
    case 'r':
      Monitor.println(readSensorsJson());
      break;
    case 'l':
      blinkLeds();
      Monitor.println("LED test done.");
      break;
    case 'z':
      chirp();
      Monitor.println("Buzzer test done.");
      break;
    case 'u':
      Monitor.print("ultrasonic_cm=");
      Monitor.println(readUltrasonicCm());
      break;
    case 'v':
      setServoAngle(90);
      setServoAngle(0);
      setServoAngle(180);
      setServoAngle(90);
      Monitor.println("Servo test done.");
      break;
    case '1':
      motorPulse(MOTOR_LEFT_FRONT);
      Monitor.println("M1 test done.");
      break;
    case '2':
      motorPulse(MOTOR_LEFT_BACK);
      Monitor.println("M2 test done.");
      break;
    case '3':
      motorPulse(MOTOR_RIGHT_FRONT);
      Monitor.println("M3 test done.");
      break;
    case '4':
      motorPulse(MOTOR_RIGHT_BACK);
      Monitor.println("M4 test done.");
      break;
    case 'f':
      driveCommand("forward", MOTOR_TEST_SPEED, MOTOR_PULSE_MS);
      Monitor.println("Forward pulse done.");
      break;
    case 'b':
      driveCommand("backward", MOTOR_TEST_SPEED, MOTOR_PULSE_MS);
      Monitor.println("Reverse pulse done.");
      break;
    case 'a':
      driveCommand("left", MOTOR_TEST_SPEED, MOTOR_PULSE_MS);
      Monitor.println("Left strafe pulse done.");
      break;
    case 'd':
      driveCommand("right", MOTOR_TEST_SPEED, MOTOR_PULSE_MS);
      Monitor.println("Right strafe pulse done.");
      break;
    case 'q':
      driveCommand("rotate_left", MOTOR_TEST_SPEED, MOTOR_PULSE_MS);
      Monitor.println("Left rotate pulse done.");
      break;
    case 'e':
      driveCommand("rotate_right", MOTOR_TEST_SPEED, MOTOR_PULSE_MS);
      Monitor.println("Right rotate pulse done.");
      break;
    case 'm':
      motorSweep();
      break;
    case 'x':
      v2StopAllCandidates();
      Monitor.println("Motors stopped.");
      break;
    case '\r':
    case '\n':
      break;
    default:
      Monitor.print("Unknown command: ");
      Monitor.println(command);
      printHelp();
      break;
  }
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

void setupPins() {
#if defined(LED3_R)
  pinMode(LED3_R, OUTPUT);
#elif defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  pinModeIfAvailable(PIN_LED_LEFT, OUTPUT);
  pinModeIfAvailable(PIN_LED_RIGHT, OUTPUT);
  pinModeIfAvailable(PIN_BUZZER, OUTPUT);
  pinModeIfAvailable(PIN_SERVO, OUTPUT);
  pinModeIfAvailable(PIN_US_TRIG, OUTPUT);

  pinModeIfAvailable(PIN_IR, INPUT);
  pinModeIfAvailable(PIN_US_ECHO, INPUT);
  pinModeIfAvailable(PIN_TRACE_L, INPUT);
  pinModeIfAvailable(PIN_TRACE_M, INPUT);
  pinModeIfAvailable(PIN_TRACE_R, INPUT);

  for (uint8_t i = 0; i < sizeof(V2_MOTOR_TX_CANDIDATES) / sizeof(V2_MOTOR_TX_CANDIDATES[0]); i++) {
    pinModeIfAvailable(V2_MOTOR_TX_CANDIDATES[i], OUTPUT);
    digitalWriteIfAvailable(V2_MOTOR_TX_CANDIDATES[i], HIGH);
  }

  digitalWriteIfAvailable(PIN_LED_LEFT, HIGH);
  digitalWriteIfAvailable(PIN_LED_RIGHT, HIGH);
  digitalWriteIfAvailable(PIN_BUZZER, LOW);
  digitalWriteIfAvailable(PIN_US_TRIG, LOW);
  boardLedWrite(false);
  v2StopAllCandidates();
}

void setup() {
  setupPins();

  const bool bridgeStarted = Bridge.begin();
  const bool monitorStarted = Monitor.begin();
  if (bridgeStarted) {
    registerBridgeMethods();
  }

  if (monitorStarted) {
    Monitor.println("ACEBOTT UNO Q Bridge app ready.");
    printHelp();
  }
}

void loop() {
  if (Monitor.available()) {
    const char command = (char)Monitor.read();
    handleMonitorCommand(command);
  }

  delay(5);
}
