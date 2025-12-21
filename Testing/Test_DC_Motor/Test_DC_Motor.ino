// Fixed DC Motor + Encoder sketch for ESP32
// - LEDC PWM (instead of analogWrite)
// - ISR is IRAM-safe and minimal (gpio_get_level)
// - Shared data protected with portMUX (critical sections)
// - Single, consistent set of pin/constants

#include <Arduino.h>
#include "driver/gpio.h"

// Pin definitions (choose one set; this uses your first set)
const int PIN_MOT_EN  = 4;   // PWM pin (LEDC)
const int PIN_MOT_IN3 = 13;  // H-Bridge IN3
const int PIN_MOT_IN4 = 14;  // H-Bridge IN4

const int PIN_ENC_A = 32;    // Encoder Channel A
const int PIN_ENC_B = 33;    // Encoder Channel B

// Encoder buffer length (moving average)
#define ENC_BUF_LEN 5

volatile long lEncoderCount = 0;               // encoder counter
volatile long arrEncBuf[ENC_BUF_LEN];         // history buffer
volatile uint8_t nEncBufIdx = 0;              // circular index

// Motor direction
bool bMotorDir = true;

// Protect shared state between ISR and main
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// LEDC (PWM) settings
const int PWM_CHANNEL   = 0;
const int PWM_FREQ      = 5000;
const int PWM_RES       = 8;                // 8-bit resolution (0..255)
const int PWM_MAX_VALUE = (1 << PWM_RES) - 1;

void IRAM_ATTR encoderISR() {
  // IRAM-safe fast reads
  int a = gpio_get_level((gpio_num_t)PIN_ENC_A);
  int b = gpio_get_level((gpio_num_t)PIN_ENC_B);

  portENTER_CRITICAL_ISR(&mux);
  if (a == b) {
    lEncoderCount++;
  } else {
    lEncoderCount--;
  }
  // update circular buffer (fast)
  arrEncBuf[nEncBufIdx] = lEncoderCount;
  nEncBufIdx++;
  if (nEncBufIdx >= ENC_BUF_LEN) nEncBufIdx = 0;
  portEXIT_CRITICAL_ISR(&mux);
}

// Return filtered (moving-average) value safely
long getFilteredEncoder() {
  long tmp[ENC_BUF_LEN];
  long sum = 0;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < ENC_BUF_LEN; ++i) tmp[i] = arrEncBuf[i];
  portEXIT_CRITICAL(&mux);

  for (int i = 0; i < ENC_BUF_LEN; ++i) sum += tmp[i];
  return sum / ENC_BUF_LEN;
}

void setMotor(int nPWM, bool bDir) {
  // set direction pins then PWM
  digitalWrite(PIN_MOT_IN3, bDir ? HIGH : LOW);
  digitalWrite(PIN_MOT_IN4, bDir ? LOW : HIGH);
  ledcWrite(PWM_CHANNEL, constrain(nPWM, 0, PWM_MAX_VALUE));
}

void stopMotor() {
  ledcWrite(PWM_CHANNEL, 0);
  digitalWrite(PIN_MOT_IN3, LOW);
  digitalWrite(PIN_MOT_IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Motor pins
  pinMode(PIN_MOT_EN, OUTPUT);
  pinMode(PIN_MOT_IN3, OUTPUT);
  pinMode(PIN_MOT_IN4, OUTPUT);

  // Encoder pins
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // init buffer/counter
  portENTER_CRITICAL(&mux);
  lEncoderCount = 0;
  nEncBufIdx = 0;
  for (int i = 0; i < ENC_BUF_LEN; ++i) arrEncBuf[i] = 0;
  portEXIT_CRITICAL(&mux);

  // PWM setup
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_MOT_EN, PWM_CHANNEL);

  // attach ISR on channel A (CHANGE)
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);

  Serial.println("DC Motor & Encoder Test Initialized");
}

void loop() {
  static unsigned long tLastDirChange = 0;
  const unsigned long T_DIR = 3000;  // 3 s
  const int PWM_VAL = 180;

  if (millis() - tLastDirChange > T_DIR) {
    bMotorDir = !bMotorDir;
    tLastDirChange = millis();
    Serial.println(bMotorDir ? "Direction: FORWARD" : "Direction: BACKWARD");
  }

  setMotor(PWM_VAL, bMotorDir);

  // safe atomic read of raw count
  long rawCnt;
  portENTER_CRITICAL(&mux);
  rawCnt = lEncoderCount;
  portEXIT_CRITICAL(&mux);

  Serial.print("Raw Cnt: ");
  Serial.print(rawCnt);
  Serial.print("\tFiltered: ");
  Serial.println(getFilteredEncoder());

  delay(250);
}