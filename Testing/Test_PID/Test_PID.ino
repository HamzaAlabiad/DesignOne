#include <ESP32Encoder.h>

// --- 1. PIN DEFINITIONS ---
#define encoderPinA 19
#define encoderPinB 18
#define motorEnable 21 // PWM Speed pin (EN)
#define motorIn1 22    // Direction pin 1 (IN1)
#define motorIn2 23    // Direction pin 2 (IN2)

// --- 2. CONSTANTS & GAINS ---
// RESOLUTION = 1 / 0.0266667 = 37.5 counts/mm
const float RESOLUTION = 37.5;
const float TARGET_DISTANCE = 44.0; // Target in mm
const float DEAD_BAND = 0.1;        // 0.1mm tolerance

// Added (recommended): velocity limiting + minimum velocity to overcome stiction
const float VEL_MAX = 120.0; // mm/s: clamp outer-loop velocity command (tune)
const float VEL_MIN = 5.0;   // mm/s: minimum velocity when outside deadband (tune)

// TUNING: Ki = Kp * T
// T = 0.09056 | Kp_vel = 2.5 | Ki_vel = 2.5 * 0.09056 = 0.2264
float kp_pos = 20;        // Outer Position Loop (P)
float kp_vel = 7;         // Inner Velocity Loop (P-part)
float ki_vel = 0.63392;   // Inner Velocity Loop (I-part)

// --- 3. STATE VARIABLES ---
ESP32Encoder Encoder;
float vel_integral = 0;
unsigned long lastTime = 0;
long lastEncoderPosition = 0;

void setup() {
  Serial.begin(115200);

  Encoder.attachFullQuad(encoderPinA, encoderPinB);
  Encoder.setCount(0);

  pinMode(motorEnable, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);

  // Added (recommended): initialize timing/state to avoid a huge first dt
  lastTime = millis();
  lastEncoderPosition = Encoder.getCount();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;

  if (dt >= 0.01) { // 100Hz Controller
    // I. DATA ACQUISITION
    long currentPulses = Encoder.getCount();
    float currentPos = currentPulses / RESOLUTION;
    float currentVel =
      (currentPulses - lastEncoderPosition) / (RESOLUTION * dt);

    // II. OUTER POSITION LOOP
    float pos_Error = TARGET_DISTANCE - currentPos;

    if (abs(pos_Error) < DEAD_BAND) {
      pos_Error = 0.0;
      vel_integral = 0.0;
    }

    float velSetpoint = kp_pos * pos_Error;

    // Added (recommended): clamp velocity setpoint
    velSetpoint = constrain(velSetpoint, -VEL_MAX, VEL_MAX);

    // Added (recommended): minimum velocity to overcome stiction
    if (abs(pos_Error) >= DEAD_BAND && abs(velSetpoint) < VEL_MIN) {
      velSetpoint = (pos_Error > 0) ? VEL_MIN : -VEL_MIN;
    }

    // III. INNER VELOCITY LOOP
    float vel_Error = velSetpoint - currentVel;
    float next_integral = vel_integral + (vel_Error * dt);
    float controlAction = (kp_vel * vel_Error) + (ki_vel * next_integral);

    // IV. SATURATION AND ANTI-WINDUP
    if (controlAction > 255) controlAction = 255;
    else if (controlAction < -255) controlAction = -255;
    else {
      if (abs(pos_Error) >= DEAD_BAND) vel_integral = next_integral;
    }

    // V. ACTUATE MOTOR
    if (abs(pos_Error) >= DEAD_BAND) {
      int pwmValue = (int)constrain(abs(controlAction), 0, 255);
      if (controlAction > 0) {
        digitalWrite(motorIn1, HIGH);
        digitalWrite(motorIn2, LOW);
      } else {
        digitalWrite(motorIn1, LOW);
        digitalWrite(motorIn2, HIGH);
      }
      analogWrite(motorEnable, pwmValue);
    } else {
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, LOW);
      analogWrite(motorEnable, 0);
    }

    // VI. LOG TO SERIAL
    Serial.print(TARGET_DISTANCE);
    Serial.print(",");
    Serial.print(currentPos);
    Serial.print(",");
    Serial.println(currentVel);

    lastTime = currentTime;
    lastEncoderPosition = currentPulses;
  }
}