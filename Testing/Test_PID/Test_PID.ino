#include <Arduino.h>
#include <ESP32Encoder.h>

// --- 1. PIN DEFINITIONS ---
#define encoderPinA 18 
#define encoderPinB 19
#define motorPWM 23
#define motorDir 22

// H-bridge mode selection
// 0 = DIR + PWM (single direction pin + PWM enable)
// 1 = L298-style: IN1 + IN2 + ENA(PWM)
#define HBRIDGE_MODE 1
// When in HBRIDGE_MODE==1 we use motorPWM as ENA, motorDir as IN1 and MOTOR_IN2 as IN2
#define MOTOR_IN2 21  // change this pin to match your wiring if needed

// PWM configuration (ESP32 LEDC)
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RESOLUTION = 8; // 8-bit (0-255)

// --- 2. CONSTANTS & GAINS ---
// Based on Scale Factor: 0.0266667 mm/count
// RESOLUTION = 1 / 0.0266667 = 37.5 counts/mm
const float RESOLUTION = 37.5; 
const float TARGET_DISTANCE = 50.0; // Target in mm
const float DEAD_BAND = 0.1;        // 0.1mm tolerance

// TUNING VALUES
// As per your document: Ki = Kp * T
// Time Constant (T) = 0.09056
// If Kp_vel = 2.5, then Ki_vel = 2.5 * 0.09056 = 0.2264
float kp_pos = 1.2;    // Outer Position Loop (P)
float kp_vel = 2.5;    // Inner Velocity Loop (P-part)
float ki_vel = 0.2264; // Inner Velocity Loop (I-part)

// --- 3. STATE VARIABLES ---
ESP32Encoder Encoder;
float vel_integral = 0;
unsigned long lastTime = 0;
long lastEncoderPosition = 0;

// ---- Motor helper functions ----
void motorInit();
void motorStop();
void motorDrive(int control);

// Implementation
void motorInit() {
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(motorPWM, PWM_CHANNEL);
  #if HBRIDGE_MODE == 1
    digitalWrite(motorDir, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  #endif
  motorStop();
}

void motorStop() {
  ledcWrite(PWM_CHANNEL, 0);
  #if HBRIDGE_MODE == 1
    digitalWrite(motorDir, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  #endif
}

void motorDrive(int control) {
  int pwm = constrain(abs(control), 0, 255);
  if (control == 0) {
    motorStop();
    return;
  }

  #if HBRIDGE_MODE == 1
    if (control >= 0) {
      digitalWrite(motorDir, HIGH); // IN1 = HIGH
      digitalWrite(MOTOR_IN2, LOW); // IN2 = LOW
    } else {
      digitalWrite(motorDir, LOW);  // IN1 = LOW
      digitalWrite(MOTOR_IN2, HIGH); // IN2 = HIGH
    }
    ledcWrite(PWM_CHANNEL, pwm);
  #else
    digitalWrite(motorDir, control >= 0 ? HIGH : LOW);
    ledcWrite(PWM_CHANNEL, pwm);
  #endif
}

void setup() {
  // Use 115200 for fast data logging, or 9600 for standard Excel Data Streamer
  Serial.begin(115200); 
  
  Encoder.attachFullQuad(encoderPinA, encoderPinB); 
  Encoder.setCount(0); 

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
  // If using H-bridge with IN2 we need to set that pin too
  #if HBRIDGE_MODE == 1
  pinMode(MOTOR_IN2, OUTPUT);
  #endif

  // Motor controller init (H-bridge support)
  motorInit();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Time step in seconds

  // Controller update interval: 10ms (100Hz)
  if (dt >= 0.01) { 
    // I. DATA ACQUISITION
    long currentPulses = Encoder.getCount();
    float currentPos = currentPulses / RESOLUTION; // mm
    float currentVel = (currentPulses - lastEncoderPosition) / (RESOLUTION * dt); // mm/s

    // II. OUTER POSITION LOOP (P)
    float pos_Error = TARGET_DISTANCE - currentPos;

    // --- DEAD-BAND LOGIC ---
    // If within 0.1mm, stop everything to prevent jitter
    if (abs(pos_Error) < DEAD_BAND) { 
      pos_Error = 0.0;     
      vel_integral = 0.0;   
      motorStop(); 
    }

    // Outer Loop Output = Inner Loop Setpoint
    float velSetpoint = kp_pos * pos_Error; 

    // III. INNER VELOCITY LOOP (PI)
    float vel_Error = velSetpoint - currentVel;

    // IV. SATURATION AND ANTI-WINDUP
    // Calculate potential integral value
    float next_integral = vel_integral + (vel_Error * dt);
    
    // PI Formula: Action = (Kp * Error) + (Ki * Integral)
    float controlAction = (kp_vel * vel_Error) + (ki_vel * next_integral);

    // Clamping to 8-bit PWM range (-255 to 255)
    if (controlAction > 255) {
        controlAction = 255;
        // Anti-windup: Don't add to integral if output is already maxed
    } else if (controlAction < -255) {
        controlAction = -255;
    } else {
        // Only integrate if we are outside the dead-band
        if (abs(pos_Error) >= DEAD_BAND) {
            vel_integral = next_integral; 
        }
    }

    // V. ACTUATE MOTOR
    if (abs(pos_Error) >= DEAD_BAND) {
        motorDrive((int)constrain(controlAction, -255, 255));
    } else {
        motorStop(); // Safety stop
    }

    // VI. LOG TO SERIAL (Format: Target, Current, Velocity)
    // Best format for Excel Data Streamer or Serial Plotter
    Serial.print(TARGET_DISTANCE); Serial.print(",");
    Serial.print(currentPos); Serial.print(",");
    Serial.println(currentVel);

    // Update state for next iteration
    lastTime = currentTime;
    lastEncoderPosition = currentPulses;
  }
}
