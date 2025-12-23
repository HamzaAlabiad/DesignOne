/*
 * Test: Quadrature Encoder (Z-axis)
 * Expected: Count increases when motor spins, displayed in Serial Monitor
 * 
 * WIRING:
 * - Encoder VCC → 3.3V or 5V
 * - Encoder GND → GND
 * - Encoder A → GPIO 27
 * - Encoder B → GPIO 26
 * 
 * LIBRARY REQUIRED:
 * - ESP32Encoder by Kevin Harrington
 *   (Install via Arduino IDE → Tools → Manage Libraries)
 */

 #include <ESP32Encoder.h>

 ESP32Encoder encoder;
 
 // Motor pins (to make encoder spin)
 #define PIN_ENA   23
 #define PIN_IN1   22
 #define PIN_IN2   21
 
 // Encoder pins
 #define PIN_ENC_A 19
 #define PIN_ENC_B 18
 
 void setup() {
   Serial.begin(115200);
   Serial.println("=================================");
   Serial.println("Encoder Test");
   Serial.println("=================================");
   
   // Setup encoder
   encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
   encoder.setCount(0);
   
   // Setup motor
   pinMode(PIN_ENA, OUTPUT);
   pinMode(PIN_IN1, OUTPUT);
   pinMode(PIN_IN2, OUTPUT);
   
   Serial.println("Encoder initialized. Spinning motor...");
   delay(1000);
 }
 
 void loop() {
   // Spin motor forward
   digitalWrite(PIN_IN1, HIGH);
   digitalWrite(PIN_IN2, LOW);
   analogWrite(PIN_ENA, 100);  // Slow speed
   
   // Print encoder count every 200ms
   Serial.print("Encoder Count: ");
   Serial.println(encoder.getCount());
   
   delay(200);
 }
 
 /*
  * EXPECTED OUTPUT:
  * Encoder Count: 0
  * Encoder Count: 45
  * Encoder Count: 89
  * Encoder Count: 134
  * ...
  * (Numbers should increase smoothly)
  * 
  * TROUBLESHOOTING:
  * 
  * Count stays at 0:
  * 1. Check encoder power (LED should be ON)
  * 2. Verify encoder wires (A → 27, B → 26)
  * 3. Check encoder is physically mounted on motor shaft
  * 4. Try swapping A and B pins in code
  * 
  * Count jumps randomly:
  * - Wires may be loose (check connections)
  * - Encoder might be damaged
  * - Try adding pullup resistors (4.7kΩ to 3.3V)
  * 
  * Count goes wrong direction:
  * - Swap A and B pins in code
  * 
  * Library not found error:
  * - Install ESP32Encoder library first
  */