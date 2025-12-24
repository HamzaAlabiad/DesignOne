/*
 * Home All 3 Axes (ESP32-B) - Simple + cleaner homing
 * Start: waits for Serial Monitor "Send"
 * Sequence: Z -> X -> Y
 *
 * Fixes:
 * - Reduces the "big debounce" (actually slow backoff time)
 * - Uses single-speed homing: stop when switch triggers and hold (no backoff/release)
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

// Step timing (per-axis): set X and Y independently so you can match speeds
// Lower microsecond values -> faster stepping (shorter pulse period).
// Pulse period = STEP_HIGH_US + STEP_LOW_US (microseconds). Frequency = 1 / period.
// Typical practical range to try: 100 .. 2000 µs (100 = fast, 2000 = slow).
// - Very low values (<=100) can be too fast for some drivers/motors and may skip steps.
// - Very high values (>2000) will be visually slow.
// To change speed: edit these constants, or implement runtime control that updates
// the *_US values from Serial commands (example idea shown below).
// Example runtime control (comment):
//   // char cmd = read from Serial; if cmd == 'X' set STEP_HIGH_US_X = value;
//   // Keep values in the recommended range and test incrementally.

// Suggested starting values to match perceived speeds (adjust to taste):
#define STEP_HIGH_US_X 100
#define STEP_LOW_US_X 100

#define STEP_HIGH_US_Y 1000
#define STEP_LOW_US_Y 1000

// Homing limits
#define STEPPER_HOME_MAX_STEPS 40000

// Z holding pwm to keep the limit switch pressed after homing
#define Z_HOLD_PWM 80

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
  // For this driver: LOW == enabled. Keep it enabled so the motor holds position
  digitalWrite(enPin, LOW); // enable driver and keep it enabled
  delay(5);
}

// (Note) We intentionally do not disable the driver after homing so the
// stepper continues to hold its position and keeps the switch pressed.

// --- New: simple, single-speed stepper homing (no backoff/release) ---
void homeStepperSimple(
  const char* name,
  int stepPin,
  int dirPin,
  int enPin,
  int limitPin,
  bool homeDirHigh,
  int highUs,
  int lowUs
) {
  Serial.printf("Homing %s...\n", name);

  // Enable driver and keep it enabled so motor will hold after stopping
  stepperEnable(enPin);

  // If already pressed, consider it homed and keep holding
  if (limitPressed(limitPin)) {
    Serial.printf("%s switch already pressed -> considered homed (holding)\n", name);
    return;
  }

  // Drive toward the switch
  digitalWrite(dirPin, homeDirHigh ? HIGH : LOW);
  delayMicroseconds(50);

  long steps = 0;
  while (!limitPressed(limitPin) && steps < STEPPER_HOME_MAX_STEPS) {
    stepOnceTimed(stepPin, highUs, lowUs);
    steps++;
    if ((steps % 800) == 0) delay(0);
  }

  if (!limitPressed(limitPin)) {
    Serial.printf("%s HOME TIMEOUT!\n", name);
    return;
  }

  // Stop stepping; driver remains enabled, so the motor holds position and
  // helps keep the switch pressed (prevents bouncy release).
  Serial.printf("%s homed (holding switch).\n\n", name);
}

void homeZ() {
  Serial.println("Homing Z...");

  // If switch already pressed, consider Z homed and keep a small holding force
  if (limitPressed(PIN_LIMIT_Z)) {
    Serial.println("Z switch already pressed -> considered homed (holding)");
    zUp(Z_HOLD_PWM); // small hold to keep switch pressed
    return;
  }

  Serial.println("Z moving up to switch...");
  zUp(Z_HOME_PWM);
  unsigned long start = millis();
  while (!limitPressed(PIN_LIMIT_Z)) {
    if (millis() - start > Z_HOME_TIMEOUT_MS) {
      Serial.println("Z HOME TIMEOUT!");
      zStop();
      return;
    }
    delay(1);
  }

  // Keep a small holding torque against the switch (do not back off)
  zUp(Z_HOLD_PWM);

  Serial.println("Z homed (holding).\n");
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
  stepperEnable(PIN_EN_X); // keep enabled to hold position after homing

  // Y stepper
  pinMode(PIN_STEP_Y, OUTPUT);
  pinMode(PIN_DIR_Y, OUTPUT);
  pinMode(PIN_EN_Y, OUTPUT);
  digitalWrite(PIN_STEP_Y, LOW);
  stepperEnable(PIN_EN_Y); // keep enabled to hold position after homing

  Serial.println("Initial limit states:");
  Serial.printf("X=%d  Y=%d  Z=%d\n",
                digitalRead(PIN_LIMIT_X),
                digitalRead(PIN_LIMIT_Y),
                digitalRead(PIN_LIMIT_Z));

  waitForSerialClick();

  homeZ();

  homeStepperSimple("X", PIN_STEP_X, PIN_DIR_X, PIN_EN_X, PIN_LIMIT_X,
                    X_HOME_DIR_HIGH, STEP_HIGH_US_X, STEP_LOW_US_X);

  homeStepperSimple("Y", PIN_STEP_Y, PIN_DIR_Y, PIN_EN_Y, PIN_LIMIT_Y,
                    Y_HOME_DIR_HIGH, STEP_HIGH_US_Y, STEP_LOW_US_Y);

  Serial.println("ALL AXES HOMED.");
}

void loop() {
  delay(1000);
}