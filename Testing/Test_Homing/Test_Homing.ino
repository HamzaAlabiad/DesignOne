/*
 * GRID MOVEMENT SYSTEM
 * Accurate XY positioning using calibrated values
 * Z-axis using YOUR EXACT WORKING PID CODE
 */

#include <ESP32Encoder.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_STEP_X 26
#define PIN_DIR_X  25
#define PIN_EN_X   27
#define PIN_LIMIT_X 33

#define PIN_STEP_Y 13
#define PIN_DIR_Y  12
#define PIN_EN_Y   14
#define PIN_LIMIT_Y 32

#define PIN_Z_ENA 21
#define PIN_Z_IN1 22
#define PIN_Z_IN2 23
#define PIN_LIMIT_Z 4

#define PIN_ENC_A 19
#define PIN_ENC_B 18

// Step timing (your perfect speeds)
#define STEP_HIGH_US_X 100
#define STEP_LOW_US_X  100
#define STEP_HIGH_US_Y 1000
#define STEP_LOW_US_Y  1000

// ============================================================================
// GRID POSITIONS (YOUR CALIBRATED VALUES)
// ============================================================================

struct Position {
  long x;
  long y;
};

Position gridPositions[17] = {
  // Pickup position (index 0)
  {0, 1230},
  
  // Slots 1-4: White products
  {1000, 10},   // Slot 1
  {5500, 10},   // Slot 2
  {1000, 310},  // Slot 3
  {5500, 310},  // Slot 4
  
  // Slots 5-8: Black products
  {10000, 10},  // Slot 5
  {14500, 10},  // Slot 6
  {10000, 310}, // Slot 7
  {14500, 310}, // Slot 8
  
  // Slots 9-12: White temp lids
  {1000, 610},  // Slot 9
  {5500, 610},  // Slot 10
  {10000, 610}, // Slot 11
  {14500, 610}, // Slot 12
  
  // Slots 13-16: Black temp lids
  {1000, 910},  // Slot 13
  {5500, 910},  // Slot 14
  {10000, 910}, // Slot 15
  {14500, 910}  // Slot 16
};

#define POS_PICKUP 0  // Pickup position index

// Current position
long currentX = 0;
long currentY = 0;

// Encoder for Z (YOUR EXACT CONSTANTS)
ESP32Encoder encoder;
const float Z_RESOLUTION = 37.5;  // Your exact value
const float Z_DEAD_BAND = 0.1;    // Your exact value

// PID state variables (YOUR EXACT VARIABLES)
float z_vel_integral = 0;
unsigned long z_lastTime = 0;
long z_lastEncoderPos = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void stepOnce(int pin, int highUs, int lowUs) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(highUs);
  digitalWrite(pin, LOW);
  delayMicroseconds(lowUs);
}

void stepperEnable(int enPin) {
  digitalWrite(enPin, LOW);
  delay(5);
}

// ============================================================================
// Z MOTOR CONTROL
// ============================================================================

void zStop() {
  analogWrite(PIN_Z_ENA, 0);
  digitalWrite(PIN_Z_IN1, LOW);
  digitalWrite(PIN_Z_IN2, LOW);
}

void zUp(uint8_t pwm) {
  digitalWrite(PIN_Z_IN1, LOW);
  digitalWrite(PIN_Z_IN2, HIGH);
  analogWrite(PIN_Z_ENA, pwm);
}

void zDown(uint8_t pwm) {
  digitalWrite(PIN_Z_IN1, HIGH);
  digitalWrite(PIN_Z_IN2, LOW);
  analogWrite(PIN_Z_ENA, pwm);
}

void homeZ() {
  Serial.println("Homing Z...");
  
  if (digitalRead(PIN_LIMIT_Z) == HIGH) {
    Serial.println("Z already home");
    zUp(80);
    encoder.setCount(0);
    z_lastEncoderPos = 0;
    z_lastTime = millis();  // Initialize timing
    return;
  }
  
  zUp(180);
  unsigned long start = millis();
  while (digitalRead(PIN_LIMIT_Z) == LOW) {
    if (millis() - start > 10000) {
      Serial.println("Z TIMEOUT!");
      zStop();
      return;
    }
    delay(1);
  }
  
  zUp(80);
  encoder.setCount(0);
  z_lastEncoderPos = 0;
  z_lastTime = millis();  // Initialize timing
  Serial.println("Z homed");
}

// YOUR EXACT WORKING PID CODE!!!
void moveZToMM(float targetMM) {
  // Your exact PID gains
  float kp_pos = 20;
  float kp_vel = 7;
  float ki_vel = 0.63392;
  
  // Your exact velocity limits
  const float VEL_MAX = 120.0;
  const float VEL_MIN = 5.0;
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < 5000) {  // 5 second timeout
    unsigned long currentTime = millis();
    float dt = (currentTime - z_lastTime) / 1000.0;
    
    if (dt >= 0.01) {  // 100Hz Controller
      // I. DATA ACQUISITION
      long currentPulses = encoder.getCount();
      float currentPos = currentPulses / Z_RESOLUTION;
      float currentVel = (currentPulses - z_lastEncoderPos) / (Z_RESOLUTION * dt);
      
      // II. OUTER POSITION LOOP
      float pos_Error = targetMM - currentPos;
      
      if (abs(pos_Error) < Z_DEAD_BAND) {
        pos_Error = 0.0;
        z_vel_integral = 0.0;
        digitalWrite(PIN_Z_IN1, LOW);
        digitalWrite(PIN_Z_IN2, LOW);
        analogWrite(PIN_Z_ENA, 0);
        return;  // SUCCESS!
      }
      
      float velSetpoint = kp_pos * pos_Error;
      
      // Clamp velocity setpoint
      velSetpoint = constrain(velSetpoint, -VEL_MAX, VEL_MAX);
      
      // Minimum velocity to overcome stiction
      if (abs(pos_Error) >= Z_DEAD_BAND && abs(velSetpoint) < VEL_MIN) {
        velSetpoint = (pos_Error > 0) ? VEL_MIN : -VEL_MIN;
      }
      
      // III. INNER VELOCITY LOOP
      float vel_Error = velSetpoint - currentVel;
      float next_integral = z_vel_integral + (vel_Error * dt);
      float controlAction = (kp_vel * vel_Error) + (ki_vel * next_integral);
      
      // IV. SATURATION AND ANTI-WINDUP
      if (controlAction > 255) {
        controlAction = 255;
      } else if (controlAction < -255) {
        controlAction = -255;
      } else {
        if (abs(pos_Error) >= Z_DEAD_BAND) {
          z_vel_integral = next_integral;
        }
      }
      
      // V. ACTUATE MOTOR
      if (abs(pos_Error) >= Z_DEAD_BAND) {
        int pwmValue = (int)constrain(abs(controlAction), 0, 255);
        if (controlAction > 0) {
          digitalWrite(PIN_Z_IN1, HIGH);
          digitalWrite(PIN_Z_IN2, LOW);
        } else {
          digitalWrite(PIN_Z_IN1, LOW);
          digitalWrite(PIN_Z_IN2, HIGH);
        }
        analogWrite(PIN_Z_ENA, pwmValue);
      } else {
        digitalWrite(PIN_Z_IN1, LOW);
        digitalWrite(PIN_Z_IN2, LOW);
        analogWrite(PIN_Z_ENA, 0);
      }
      
      // VI. UPDATE STATE
      z_lastTime = currentTime;
      z_lastEncoderPos = currentPulses;
    }
  }
  
  // Timeout - stop motor
  digitalWrite(PIN_Z_IN1, LOW);
  digitalWrite(PIN_Z_IN2, LOW);
  analogWrite(PIN_Z_ENA, 0);
  Serial.println("Z movement timeout!");
}

// ============================================================================
// HOMING XY
// ============================================================================

void homeX() {
  Serial.println("Homing X...");
  
  if (digitalRead(PIN_LIMIT_X) == HIGH) {
    Serial.println("X already home");
    currentX = 0;
    return;
  }
  
  digitalWrite(PIN_DIR_X, LOW);
  stepperEnable(PIN_EN_X);
  
  long steps = 0;
  while (digitalRead(PIN_LIMIT_X) == LOW && steps < 40000) {
    stepOnce(PIN_STEP_X, STEP_HIGH_US_X, STEP_LOW_US_X);
    steps++;
    if (steps % 800 == 0) delay(0);
  }
  
  currentX = 0;
  Serial.println("X homed");
}

void homeY() {
  Serial.println("Homing Y...");
  
  if (digitalRead(PIN_LIMIT_Y) == HIGH) {
    Serial.println("Y already home");
    currentY = 0;
    return;
  }
  
  digitalWrite(PIN_DIR_Y, HIGH);
  stepperEnable(PIN_EN_Y);
  
  long steps = 0;
  while (digitalRead(PIN_LIMIT_Y) == LOW && steps < 40000) {
    stepOnce(PIN_STEP_Y, STEP_HIGH_US_Y, STEP_LOW_US_Y);
    steps++;
    if (steps % 800 == 0) delay(0);
  }
  
  currentY = 0;
  Serial.println("Y homed");
}

void homeAll() {
  homeZ();
  delay(500);
  homeX();
  delay(500);
  homeY();
  delay(500);
  Serial.println("ALL AXES HOMED!");
}

// ============================================================================
// XY MOVEMENT FUNCTIONS
// ============================================================================

void moveXAbsolute(long targetX) {
  long steps = targetX - currentX;
  
  if (steps == 0) {
    Serial.println("X already at target");
    return;
  }
  
  Serial.print("Moving X from ");
  Serial.print(currentX);
  Serial.print(" to ");
  Serial.println(targetX);
  
  stepperEnable(PIN_EN_X);
  digitalWrite(PIN_DIR_X, steps > 0 ? HIGH : LOW);
  delayMicroseconds(50);
  
  for (long i = 0; i < abs(steps); i++) {
    stepOnce(PIN_STEP_X, STEP_HIGH_US_X, STEP_LOW_US_X);
    if (i % 800 == 0) delay(0);
  }
  
  currentX = targetX;
  Serial.print("X arrived at ");
  Serial.println(currentX);
}

void moveYAbsolute(long targetY) {
  long steps = targetY - currentY;
  
  if (steps == 0) {
    Serial.println("Y already at target");
    return;
  }
  
  Serial.print("Moving Y from ");
  Serial.print(currentY);
  Serial.print(" to ");
  Serial.println(targetY);
  
  stepperEnable(PIN_EN_Y);
  digitalWrite(PIN_DIR_Y, steps > 0 ? LOW : HIGH);
  delayMicroseconds(50);
  
  for (long i = 0; i < abs(steps); i++) {
    stepOnce(PIN_STEP_Y, STEP_HIGH_US_Y, STEP_LOW_US_Y);
    if (i % 800 == 0) delay(0);
  }
  
  currentY = targetY;
  Serial.print("Y arrived at ");
  Serial.println(currentY);
}

void moveXYToSlot(int slot) {
  if (slot < 0 || slot > 16) {
    Serial.println("ERROR: Invalid slot!");
    return;
  }
  
  Serial.print("Moving to slot ");
  Serial.println(slot);
  
  Position target = gridPositions[slot];
  
  // Move X first, then Y (safer - avoids collisions)
  moveXAbsolute(target.x);
  delay(200);
  moveYAbsolute(target.y);
  delay(200);
  
  Serial.println("Arrived at slot!");
}

void moveToPickup() {
  Serial.println("Moving to PICKUP position...");
  moveXYToSlot(POS_PICKUP);
}

void moveToHome() {
  Serial.println("Returning to HOME (0,0)...");
  moveYAbsolute(0);
  delay(200);
  moveXAbsolute(0);
  delay(200);
  Serial.println("At home position");
}

// ============================================================================
// PICK & PLACE FUNCTIONS
// ============================================================================

void pickFromSlot(int slot) {
  Serial.print("Picking from slot ");
  Serial.println(slot);
  
  moveXYToSlot(slot);
  delay(500);
  
  moveZToMM(30);  // Your calibrated value
  delay(1000);
  
  Serial.println("VACUUM ON");
  delay(1000);
  
  moveZToMM(0);
  delay(500);
  
  Serial.println("Picked!");
}

void placeAtSlot(int slot) {
  Serial.print("Placing at slot ");
  Serial.println(slot);
  
  moveXYToSlot(slot);
  delay(500);
  
  moveZToMM(23);
  delay(1000);
  
  Serial.println("VACUUM OFF");
  delay(1000);
  
  moveZToMM(0);
  delay(500);
  
  Serial.println("Placed!");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===========================================");
  Serial.println("GRID MOVEMENT SYSTEM");
  Serial.println("===========================================");
  
  // Init pins
  pinMode(PIN_LIMIT_X, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Y, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Z, INPUT_PULLUP);
  
  pinMode(PIN_Z_ENA, OUTPUT);
  pinMode(PIN_Z_IN1, OUTPUT);
  pinMode(PIN_Z_IN2, OUTPUT);
  zStop();
  
  pinMode(PIN_STEP_X, OUTPUT);
  pinMode(PIN_DIR_X, OUTPUT);
  pinMode(PIN_EN_X, OUTPUT);
  digitalWrite(PIN_STEP_X, LOW);
  
  pinMode(PIN_STEP_Y, OUTPUT);
  pinMode(PIN_DIR_Y, OUTPUT);
  pinMode(PIN_EN_Y, OUTPUT);
  digitalWrite(PIN_STEP_Y, LOW);
  
  // Init encoder
  encoder.attachFullQuad(PIN_ENC_A, PIN_ENC_B);
  encoder.setCount(0);
  z_lastTime = millis();
  z_lastEncoderPos = 0;
  
  Serial.println("\nPress SEND to home all axes...");
  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read();
  
  homeAll();
  
  Serial.println("\n===========================================");
  Serial.println("READY FOR TESTING!");
  Serial.println("===========================================");
  Serial.println("Commands:");
  Serial.println("  g<slot> - Go to slot (e.g., g0, g15)");
  Serial.println("  p - Go to pickup");
  Serial.println("  h - Go to home");
  Serial.println("  pick<slot> - Pick from slot");
  Serial.println("  place<slot> - Place at slot");
  Serial.println();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("g")) {
      int slot = cmd.substring(1).toInt();
      moveXYToSlot(slot);
      
    } else if (cmd == "p") {
      moveToPickup();
      
    } else if (cmd == "h") {
      moveToHome();
      
    } else if (cmd.startsWith("pick")) {
      int slot = cmd.substring(4).toInt();
      pickFromSlot(slot);
      
    } else if (cmd.startsWith("place")) {
      int slot = cmd.substring(5).toInt();
      placeAtSlot(slot);
      
    } else {
      Serial.println("Unknown command!");
    }
  }
  
  delay(100);
}