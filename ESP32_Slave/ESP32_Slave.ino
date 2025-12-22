//ESP32 Master Pin Definitions
// SENSORS
#define PIN_IR_FEEDER       19 // IR sensor at feeder
#define PIN_IR_COLOR        23  // IR sensor for color
#define PIN_LASER_RX        32  // Laser receiver
#define PIN_LIMIT_SORT      13  // Limit switch at sort position

// ACTUATORS (2-Channel Relay)
#define PIN_RELAY_FEEDER    26  // Relay 1: Pneumatic cylinder
#define PIN_RELAY_VACUUM    25  // Relay 2: Vacuum pump

// LCD I2C (Fixed pins for I2C)
#define PIN_LCD_SDA         21  // I2C Data
#define PIN_LCD_SCL         22  // I2C Clock


//ESP32 Slave Pin Definitions
// STEPPER X-AXIS
#define PIN_STEP_X          13
#define PIN_DIR_X           12
#define PIN_EN_X            14
#define PIN_LIMIT_X         33

// STEPPER Y-AXIS
#define PIN_STEP_Y          27
#define PIN_DIR_Y           26
#define PIN_EN_Y            25
#define PIN_LIMIT_Y         32

// DC MOTOR Z-AXIS
#define PIN_Z_ENA           23  // PWM speed
#define PIN_Z_IN1           22  // Direction bit 1
#define PIN_Z_IN2           21  // Direction bit 2
#define PIN_Z_ENC_A         19  // Encoder A
#define PIN_Z_ENC_B         18  // Encoder B
#define PIN_LIMIT_Z         4
