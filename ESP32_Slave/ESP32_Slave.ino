/*
 * ESP32 SLAVE - INTEGRATED SIMPLE CODE
 * Homing, Motion Control, ESP-NOW
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Encoder.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// Limit switches
#define PIN_LIMIT_X 33
#define PIN_LIMIT_Y 32
#define PIN_LIMIT_Z 4

// Z motor (L298N)
#define PIN_Z_ENA 21
#define PIN_Z_IN1 22
#define PIN_Z_IN2 23

// Y stepper
#define PIN_STEP_Y 13
#define PIN_DIR_Y  12
#define PIN_EN_Y   14

// X stepper
#define PIN_STEP_X 26
#define PIN_DIR_X  25
#define PIN_EN_X   27

// Encoder
#define PIN_ENC_A  19
#define PIN_ENC_B  18

// ============================================================================
// OBJECTS
// ============================================================================
ESP32Encoder encoder;

// ============================================================================
// ESP-NOW
// ============================================================================
struct ProductData {
  bool isBase;
  bool isWhite;
};

ProductData receivedProduct;
bool newProductAvailable = false;

// ============================================================================
// ENCODER CONTROL
// ============================================================================
const float RESOLUTION = 37.5;
const float DEAD_BAND = 0.1;
float vel_integral = 0;
unsigned long lastTime = 0;
long lastEncoderPos = 0;

// ============================================================================
// FUNCTIONS
// ============================================================================

// -------------------- STEPPER --------------------
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

// -------------------- DC MOTOR --------------------
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

// -------------------- HOMING --------------------
void homeZ() {
  Serial.println("Homing Z...");
  
  if (digitalRead(PIN_LIMIT_Z) == HIGH) {
    Serial.println("Z already home");
    zUp(80);
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
  
  zUp(80);  // Hold
  Serial.println("Z homed");
}

void homeX() {
  Serial.println("Homing X...");
  
  if (digitalRead(PIN_LIMIT_X) == HIGH) {
    Serial.println("X already home");
    return;
  }
  
  digitalWrite(PIN_DIR_X, LOW);
  stepperEnable(PIN_EN_X);
  
  long steps = 0;
  while (digitalRead(PIN_LIMIT_X) == LOW && steps < 40000) {
    stepOnce(PIN_STEP_X, 100, 100);
    steps++;
    if (steps % 800 == 0) delay(0);
  }
  
  Serial.println("X homed");
}

void homeY() {
  Serial.println("Homing Y...");
  
  if (digitalRead(PIN_LIMIT_Y) == HIGH) {
    Serial.println("Y already home");
    return;
  }
  
  digitalWrite(PIN_DIR_Y, HIGH);
  stepperEnable(PIN_EN_Y);
  
  long steps = 0;
  while (digitalRead(PIN_LIMIT_Y) == LOW && steps < 40000) {
    stepOnce(PIN_STEP_Y, 1000, 1000);
    steps++;
    if (steps % 800 == 0) delay(0);
  }
  
  Serial.println("Y homed");
}

void homeAll() {
  homeZ();
  delay(500);
  homeX();
  delay(500);
  homeY();
  delay(500);
  Serial.println("ALL HOMED!");
}

// -------------------- MOTION --------------------
void moveZToMM(float targetMM) {
  float kp_pos = 20;
  float kp_vel = 7;
  float ki_vel = 0.63392;
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < 5000) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    
    if (dt >= 0.01) {
      long currentPulses = encoder.getCount();
      float currentPos = currentPulses / RESOLUTION;
      float currentVel = (currentPulses - lastEncoderPos) / (RESOLUTION * dt);
      
      float pos_Error = targetMM - currentPos;
      
      if (abs(pos_Error) < DEAD_BAND) {
        zStop();
        vel_integral = 0;
        return;
      }
      
      float velSetpoint = kp_pos * pos_Error;
      velSetpoint = constrain(velSetpoint, -120, 120);
      
      float vel_Error = velSetpoint - currentVel;
      float next_integral = vel_integral + (vel_Error * dt);
      float controlAction = (kp_vel * vel_Error) + (ki_vel * next_integral);
      
      controlAction = constrain(controlAction, -255, 255);
      vel_integral = next_integral;
      
      int pwm = (int)abs(controlAction);
      if (controlAction > 0) {
        digitalWrite(PIN_Z_IN1, HIGH);
        digitalWrite(PIN_Z_IN2, LOW);
      } else {
        digitalWrite(PIN_Z_IN1, LOW);
        digitalWrite(PIN_Z_IN2, HIGH);
      }
      analogWrite(PIN_Z_ENA, pwm);
      
      lastTime = currentTime;
      lastEncoderPos = currentPulses;
    }
  }
  
  zStop();
}

void moveXSteps(long steps) {
  digitalWrite(PIN_DIR_X, steps > 0 ? HIGH : LOW);
  stepperEnable(PIN_EN_X);
  
  for (long i = 0; i < abs(steps); i++) {
    stepOnce(PIN_STEP_X, 100, 100);
    if (i % 800 == 0) delay(0);
  }
}

void moveYSteps(long steps) {
  digitalWrite(PIN_DIR_Y, steps > 0 ? LOW : HIGH);
  stepperEnable(PIN_EN_Y);
  
  for (long i = 0; i < abs(steps); i++) {
    stepOnce(PIN_STEP_Y, 1000, 1000);
    if (i % 800 == 0) delay(0);
  }
}

// -------------------- PICK & PLACE --------------------
void pickupProduct() {
  Serial.println("Picking up...");
  
  // Go to pickup position (adjust these values!)
  moveXSteps(5000);   // Move X
  delay(500);
  moveYSteps(2000);   // Move Y
  delay(500);
  
  // Lower Z
  moveZToMM(30);
  delay(1000);
  
  // Vacuum ON (controlled by Master - simulate here)
  Serial.println("Vacuum ON");
  delay(1000);
  
  // Raise Z
  moveZToMM(0);
  delay(500);
}

void placeProduct(bool isWhite) {
  Serial.println("Placing...");
  
  // Go to placement position (adjust these!)
  if (isWhite) {
    moveXSteps(3000);
    moveYSteps(1000);
  } else {
    moveXSteps(6000);
    moveYSteps(1000);
  }
  
  delay(500);
  
  // Lower Z
  moveZToMM(30);
  delay(1000);
  
  // Vacuum OFF
  Serial.println("Vacuum OFF");
  delay(1000);
  
  // Raise Z
  moveZToMM(0);
  delay(500);
}

void returnHome() {
  Serial.println("Returning home...");
  moveZToMM(0);
  delay(500);
  
  // Return X, Y to home
  moveXSteps(-20000);  // Move back
  moveYSteps(-20000);
}

// -------------------- ESP-NOW --------------------
void onReceiveSlave(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(ProductData)) {
    memcpy(&receivedProduct, data, sizeof(ProductData));
    newProductAvailable = true;
    
    Serial.println("Received product data:");
    Serial.print("  Type: ");
    Serial.println(receivedProduct.isBase ? "Base" : "Lid");
    Serial.print("  Color: ");
    Serial.println(receivedProduct.isWhite ? "White" : "Black");
  }
}

void initESPNOW() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW FAILED!");
    while(1) delay(1000);
  }
  
  esp_now_register_recv_cb(onReceiveSlave);
  
  Serial.println("ESP-NOW Ready");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 SLAVE STARTING...");
  
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
  lastTime = millis();
  lastEncoderPos = 0;
  
  // Init ESP-NOW
  initESPNOW();
  
  // Print MAC
  uint8_t myMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
  Serial.print("Slave MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", myMac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  // Home all axes
  Serial.println("\nPress SEND to start homing...");
  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read();
  
  homeAll();
  
  Serial.println("\nREADY FOR PRODUCTS!");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  if (newProductAvailable) {
    newProductAvailable = false;
    
    Serial.println("\n=== NEW PRODUCT ===");
    
    // Pickup
    pickupProduct();
    
    // Place
    placeProduct(receivedProduct.isWhite);
    
    // Return home
    returnHome();
    
    Serial.println("=== CYCLE COMPLETE ===\n");
  }
  
  delay(100);
}