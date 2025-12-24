#define STEP_PIN 26
#define DIR_PIN 25
#define ENABLE_PIN 27 // ENABLE is controlled by code

void setup() {
  // Configure pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Driver initialization
  digitalWrite(ENABLE_PIN, HIGH); // Disable driver during setup (safe)
  delay(10);                      // Allow signals to settle
  digitalWrite(ENABLE_PIN, LOW);  // ENABLE driver (LOW = ON)

  // Set initial direction
  digitalWrite(DIR_PIN, HIGH);
}

void loop() {
  // Move motor forward
  for (int i = 0; i < 1600; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800); // Slow, safe pulse
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }

  delay(1000);

  // Reverse direction
  digitalWrite(DIR_PIN, !digitalRead(DIR_PIN));
}