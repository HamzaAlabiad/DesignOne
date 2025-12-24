/*
 * Home All 3 Axes (ESP32-B) - Simple + cleaner homing
 * Start: waits for Serial Monitor "Send"
 * Sequence: Z -> X -> Y
 *
 * Fixes:
 * - Reduces the "big debounce" (actually slow backoff time)
 * - Adds 2-stage homing (fast then slow) for better switch behavior
 *
 * If limit logic is reversed, change:
 *   LIMIT_PRESSED_IS_HIGH (1 <-> 0)
 */

// -------------------------
// PINS (your current pins from the pasted sketch)
// -------------------------

// Limit switches
#define PIN_LIMIT_X 33
#define PIN_LIMIT_Y 32
#define PIN_LIMIT_Z 4

// Z DC motor (L298N)  (NOTE: you swapped these vs earlier tests; keep as YOU wired)
#define PIN_Z_ENA 21
#define PIN_Z_IN1 22
#define PIN_Z_IN2 23

// Y stepper
#define PIN_STEP_Y 13
#define PIN_DIR_Y 12
#define PIN_EN_Y 14

// X stepper
#define PIN_STEP_X 26
#define PIN_DIR_X 25
#define PIN_EN_X 27

// -------------------------
// SETTINGS
// -------------------------

#define LIMIT_PRESSED_IS_HIGH 1

#define X_HOME_DIR_HIGH false
#define Y_HOME_DIR_HIGH true

#define Z_UP_IN1_HIGH false

// Z
#define Z_HOME_PWM 180
#define Z_BACKOFF_MS 150
#define Z_HOME_TIMEOUT_MS 10000

// Step timing (same for both; if you must tune visually, adjust X vs Y)
#define X_STEP_HIGH_US_FAST 500
#define X_STEP_LOW_US_FAST 500
#define X_STEP_HIGH_US_SLOW 1000
#define X_STEP_LOW_US_SLOW 1000

#define Y_STEP_HIGH_US_FAST 500
#define Y_STEP_LOW_US_FAST 500
#define Y_STEP_HIGH_US_SLOW 1000
#define Y_STEP_LOW_US_SLOW 1000

// Homing limits
#define STEPPER_HOME_MAX_STEPS 40000

// Backoff: smaller + faster (fixes your “huge debounce” feeling)
#define STEPPER_BACKOFF_STEPS 80

// -------------------------
// Helpers
// -------------------------

bool limitPressed(int pin) {
  int v = digitalRead(pin);
#if LIMIT_PRESSED_IS_HIGH
  return (v == HIGH);
#else
  return (v == LOW);
#endif
}

void waitForSerialClick() {
  Serial.println("\n=================================");
  Serial.println("READY: Press Send in Serial Monitor to start homing");
  Serial.println("=================================\n");

  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(50);
  while (Serial.available()) Serial.read();

  Serial.println("Starting homing...\n");
  delay(200);
}

// ----- Z motor control -----
void zStop() {
  analogWrite(PIN_Z_ENA, 0);
  digitalWrite(PIN_Z_IN1, LOW);
  digitalWrite(PIN_Z_IN2, LOW);
}

void zUp(uint8_t pwm) {
  if (Z_UP_IN1_HIGH) {
    digitalWrite(PIN_Z_IN1, HIGH);
    digitalWrite(PIN_Z_IN2, LOW);
  } else {
    digitalWrite(PIN_Z_IN1, LOW);
    digitalWrite(PIN_Z_IN2, HIGH);
  }
  analogWrite(PIN_Z_ENA, pwm);
}

void zDown(uint8_t pwm) {
  if (Z_UP_IN1_HIGH) {
    digitalWrite(PIN_Z_IN1, LOW);
    digitalWrite(PIN_Z_IN2, HIGH);
  } else {
    digitalWrite(PIN_Z_IN1, HIGH);
    digitalWrite(PIN_Z_IN2, LOW);
  }
  analogWrite(PIN_Z_ENA, pwm);
}

// ----- Stepper control -----
void stepOnceTimed(int stepPin, int highUs, int lowUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(highUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(lowUs);
}

void stepperEnable(int enPin) {
  // Your driver behavior: HIGH disable, LOW enable
  digitalWrite(enPin, HIGH);
  delay(10);
  digitalWrite(enPin, LOW);
}

void homeZ() {
  Serial.println("Homing Z...");

  // Back off if already pressed
  if (limitPressed(PIN_LIMIT_Z)) {
    Serial.println("Z switch already pressed -> backing off...");
    zDown(Z_HOME_PWM);
    unsigned long t0 = millis();
    while (limitPressed(PIN_LIMIT_Z) && (millis() - t0 < 2000)) delay(1);
    zStop();
    delay(100);
  }

  Serial.println("Z moving up to switch...");
  zUp(Z_HOME_PWM);
  unsigned long start = millis();
  while (!limitPressed(PIN_LIMIT_Z)) {
    if (millis() - start > Z_HOME_TIMEOUT_MS) {
      Serial.println("Z HOME TIMEOUT!");
      break;
    }
    delay(1);
  }
  zStop();
  delay(50);

  // Small backoff (faster)
  zDown(Z_HOME_PWM);
  delay(Z_BACKOFF_MS);
  zStop();

  Serial.println("Z homed.\n");
}

void stepperBackoff(
  int stepPin,
  int dirPin,
  bool homeDirHigh,
  int backoffSteps,
  int highUs,
  int lowUs
) {
  // Move away from switch
  digitalWrite(dirPin, homeDirHigh ? LOW : HIGH);
  delayMicroseconds(50);

  for (int i = 0; i < backoffSteps; i++) {
    stepOnceTimed(stepPin, highUs, lowUs);
    if ((i % 400) == 0) delay(0);
  }
}

void homeStepper2Stage(
  const char* name,
  int stepPin,
  int dirPin,
  int enPin,
  int limitPin,
  bool homeDirHigh,
  int fastHighUs,
  int fastLowUs,
  int slowHighUs,
  int slowLowUs
) {
  Serial.printf("Homing %s...\n", name);

  stepperEnable(enPin);

  // If already pressed, back off first
  if (limitPressed(limitPin)) {
    Serial.printf("%s switch already pressed -> backing off...\n", name);
    stepperBackoff(stepPin, dirPin, homeDirHigh, STEPPER_BACKOFF_STEPS * 2,
                   fastHighUs, fastLowUs);
  }

  // -------- Stage 1: FAST approach --------
  Serial.printf("%s fast approach...\n", name);
  digitalWrite(dirPin, homeDirHigh ? HIGH : LOW);
  delayMicroseconds(50);

  long steps = 0;
  while (!limitPressed(limitPin) && steps < STEPPER_HOME_MAX_STEPS) {
    stepOnceTimed(stepPin, fastHighUs, fastLowUs);
    steps++;
    if ((steps % 800) == 0) delay(0);
  }

  if (!limitPressed(limitPin)) {
    Serial.printf("%s HOME TIMEOUT (fast stage)!\n", name);
    return;
  }

  // Quick settle check (not “big debounce”, just stability)
  delay(5);

  // Back off a little (fast)
  stepperBackoff(stepPin, dirPin, homeDirHigh, STEPPER_BACKOFF_STEPS,
                 fastHighUs, fastLowUs);

  // -------- Stage 2: SLOW approach --------
  Serial.printf("%s slow approach...\n", name);
  digitalWrite(dirPin, homeDirHigh ? HIGH : LOW);
  delayMicroseconds(50);

  steps = 0;
  while (!limitPressed(limitPin) && steps < STEPPER_HOME_MAX_STEPS) {
    stepOnceTimed(stepPin, slowHighUs, slowLowUs);
    steps++;
    if ((steps % 800) == 0) delay(0);
  }

  if (!limitPressed(limitPin)) {
    Serial.printf("%s HOME TIMEOUT (slow stage)!\n", name);
    return;
  }

  delay(5);

  // Final backoff to release switch (fast)
  stepperBackoff(stepPin, dirPin, homeDirHigh, STEPPER_BACKOFF_STEPS,
                 fastHighUs, fastLowUs);

  Serial.printf("%s homed.\n\n", name);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Limit switches
  pinMode(PIN_LIMIT_X, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Y, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Z, INPUT_PULLUP);

  // Z motor
  pinMode(PIN_Z_ENA, OUTPUT);
  pinMode(PIN_Z_IN1, OUTPUT);
  pinMode(PIN_Z_IN2, OUTPUT);
  zStop();

  // X stepper
  pinMode(PIN_STEP_X, OUTPUT);
  pinMode(PIN_DIR_X, OUTPUT);
  pinMode(PIN_EN_X, OUTPUT);
  digitalWrite(PIN_STEP_X, LOW);

  // Y stepper
  pinMode(PIN_STEP_Y, OUTPUT);
  pinMode(PIN_DIR_Y, OUTPUT);
  pinMode(PIN_EN_Y, OUTPUT);
  digitalWrite(PIN_STEP_Y, LOW);

  Serial.println("Initial limit states:");
  Serial.printf("X=%d  Y=%d  Z=%d\n",
                digitalRead(PIN_LIMIT_X),
                digitalRead(PIN_LIMIT_Y),
                digitalRead(PIN_LIMIT_Z));

  waitForSerialClick();

  homeZ();

  homeStepper2Stage("X", PIN_STEP_X, PIN_DIR_X, PIN_EN_X, PIN_LIMIT_X,
                    X_HOME_DIR_HIGH, X_STEP_HIGH_US_FAST, X_STEP_LOW_US_FAST,
                    X_STEP_HIGH_US_SLOW, X_STEP_LOW_US_SLOW);

  homeStepper2Stage("Y", PIN_STEP_Y, PIN_DIR_Y, PIN_EN_Y, PIN_LIMIT_Y,
                    Y_HOME_DIR_HIGH, Y_STEP_HIGH_US_FAST, Y_STEP_LOW_US_FAST,
                    Y_STEP_HIGH_US_SLOW, Y_STEP_LOW_US_SLOW);

  Serial.println("ALL AXES HOMED.");
}

void loop() {
  delay(1000);
}