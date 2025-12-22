/*
 * Test: Stepper Motor (X or Y axis)
 * Expected: Motor rotates 200 steps forward, then 200 steps backward, repeat
 * 
 * WIRING:
 * - ESP32 Pin 19 → Driver STEP
 * - ESP32 Pin 18 → Driver DIR
 * - ESP32 Pin 5 → Driver EN
 * - ESP32 GND → Driver GND
 * - 12V Power → Driver VMOT/GND
 */

 #define PIN_STEP  19
 #define PIN_DIR   18
 #define PIN_EN    5
 
 void setup() {
   Serial.begin(115200);
   Serial.println("=================================");
   Serial.println("Stepper Motor Test");
   Serial.println("=================================");
   
   pinMode(PIN_STEP, OUTPUT);
   pinMode(PIN_DIR, OUTPUT);
   pinMode(PIN_EN, OUTPUT);
   
   // Enable motor (active LOW)
   digitalWrite(PIN_EN, LOW);
   
   Serial.println("Motor enabled. Starting test...");
   delay(1000);
 }
 
 void loop() {
   // ====== MOVE FORWARD ======
   Serial.println("Moving FORWARD (200 steps)...");
   digitalWrite(PIN_DIR, HIGH);  // Set direction
   
   for(int i = 0; i < 200; i++) {
     digitalWrite(PIN_STEP, HIGH);
     delayMicroseconds(2000);  // Adjust if too fast/slow
     digitalWrite(PIN_STEP, LOW);
     delayMicroseconds(2000);
   }
   
   Serial.println("Forward complete. Pausing...");
   delay(1000);
   
   // ====== MOVE BACKWARD ======
   Serial.println("Moving BACKWARD (200 steps)...");
   digitalWrite(PIN_DIR, LOW);  // Reverse direction
   
   for(int i = 0; i < 200; i++) {
     digitalWrite(PIN_STEP, HIGH);
     delayMicroseconds(2000);
     digitalWrite(PIN_STEP, LOW);
     delayMicroseconds(2000);
   }
   
   Serial.println("Backward complete. Pausing...");
   delay(1000);
 }
 
 /*
  * TROUBLESHOOTING:
  * 
  * Motor doesn't move:
  * 1. Check driver power LED (should be ON)
  * 2. Verify 12V connected to driver
  * 3. Check ESP32 GND connected to driver GND
  * 4. Try changing EN to HIGH (some drivers are active HIGH)
  * 5. Increase delayMicroseconds to 5000 (slower speed)
  * 6. Swap motor wire pairs (try different coil combinations)
  * 
  * Motor vibrates but doesn't turn:
  * - Increase delayMicroseconds (motor too fast)
  * - Check motor current setting on driver (potentiometer)
  * 
  * Motor turns wrong direction:
  * - Flip DIR signal (HIGH ↔ LOW)
  */