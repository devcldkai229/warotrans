#include <Arduino.h>
#include <driver/gpio.h>

// Encoder wiring:
//   Left:  C2 -> GPIO 8,  C1 -> GPIO 9
//   Right: C2 -> GPIO 10, C1 -> GPIO 11
constexpr uint8_t LEFT_ENCODER_C2_PIN = 8;
constexpr uint8_t LEFT_ENCODER_C1_PIN = 9;
constexpr uint8_t RIGHT_ENCODER_C2_PIN = 10;
constexpr uint8_t RIGHT_ENCODER_C1_PIN = 11;

// Doi dau nay khi wiring thuc te cho chieu tick nguoc voi quy uoc mong muon.
constexpr int8_t LEFT_ENCODER_SIGN = -1;
constexpr int8_t RIGHT_ENCODER_SIGN = 1;

constexpr uint32_t SERIAL_BAUD = 921600;
constexpr uint32_t REPORT_PERIOD_MS = 20;

volatile int32_t leftTicks = 0;
volatile int32_t rightTicks = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Index: previous_state << 2 | current_state.
// State bits are ordered as C2:C1. Valid transitions are decoded as x4.
DRAM_ATTR static const int8_t QUADRATURE_DELTA[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

IRAM_ATTR uint8_t readEncoderState(uint8_t c2Pin, uint8_t c1Pin) {
  const uint8_t c2 = static_cast<uint8_t>(
      gpio_get_level(static_cast<gpio_num_t>(c2Pin)));
  const uint8_t c1 = static_cast<uint8_t>(
      gpio_get_level(static_cast<gpio_num_t>(c1Pin)));
  return static_cast<uint8_t>((c2 << 1) | c1);
}

IRAM_ATTR void updateEncoder(volatile int32_t& ticks,
                             volatile uint8_t& previousState,
                             uint8_t c2Pin,
                             uint8_t c1Pin) {
  portENTER_CRITICAL_ISR(&encoderMux);

  const uint8_t currentState = readEncoderState(c2Pin, c1Pin);
  const uint8_t transition =
      static_cast<uint8_t>((previousState << 2) | currentState);
  ticks += QUADRATURE_DELTA[transition];
  previousState = currentState;

  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onLeftC2Change() {
  updateEncoder(leftTicks, leftState, LEFT_ENCODER_C2_PIN,
                LEFT_ENCODER_C1_PIN);
}

void IRAM_ATTR onLeftC1Change() {
  updateEncoder(leftTicks, leftState, LEFT_ENCODER_C2_PIN,
                LEFT_ENCODER_C1_PIN);
}

void IRAM_ATTR onRightC2Change() {
  updateEncoder(rightTicks, rightState, RIGHT_ENCODER_C2_PIN,
                RIGHT_ENCODER_C1_PIN);
}

void IRAM_ATTR onRightC1Change() {
  updateEncoder(rightTicks, rightState, RIGHT_ENCODER_C2_PIN,
                RIGHT_ENCODER_C1_PIN);
}

void resetTicks() {
  portENTER_CRITICAL(&encoderMux);
  leftTicks = 0;
  rightTicks = 0;
  portEXIT_CRITICAL(&encoderMux);
}

void copyTicks(int32_t& left, int32_t& right) {
  portENTER_CRITICAL(&encoderMux);
  left = leftTicks;
  right = rightTicks;
  portEXIT_CRITICAL(&encoderMux);
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    if (Serial.read() == 'R') {
      resetTicks();
    }
  }
}

void setup() {
  pinMode(LEFT_ENCODER_C2_PIN, INPUT);
  pinMode(LEFT_ENCODER_C1_PIN, INPUT);
  pinMode(RIGHT_ENCODER_C2_PIN, INPUT);
  pinMode(RIGHT_ENCODER_C1_PIN, INPUT);

  portENTER_CRITICAL(&encoderMux);
  leftState = readEncoderState(LEFT_ENCODER_C2_PIN, LEFT_ENCODER_C1_PIN);
  rightState =
      readEncoderState(RIGHT_ENCODER_C2_PIN, RIGHT_ENCODER_C1_PIN);
  leftTicks = 0;
  rightTicks = 0;
  portEXIT_CRITICAL(&encoderMux);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_C2_PIN),
                  onLeftC2Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_C1_PIN),
                  onLeftC1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_C2_PIN),
                  onRightC2Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_C1_PIN),
                  onRightC1Change, CHANGE);

  Serial.begin(SERIAL_BAUD);
}

void loop() {
  processSerialCommands();

  static uint32_t nextReportMs = millis() + REPORT_PERIOD_MS;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextReportMs) >= 0) {
    nextReportMs += REPORT_PERIOD_MS;

    int32_t leftSnapshot = 0;
    int32_t rightSnapshot = 0;
    copyTicks(leftSnapshot, rightSnapshot);

    const long outputLeft =
        static_cast<long>(leftSnapshot * LEFT_ENCODER_SIGN);
    const long outputRight =
        static_cast<long>(rightSnapshot * RIGHT_ENCODER_SIGN);
    Serial.printf("O %ld %ld %lu\n", outputLeft, outputRight,
                  static_cast<unsigned long>(now));
  }
}
