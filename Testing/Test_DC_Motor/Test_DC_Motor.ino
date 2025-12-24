/*
 * Test: DC Motor with L298N (Z-axis)
 * Expected: Motor spins forward, stops, spins backward, stops, repeat
 * 
 * WIRING:
 * - ESP32 Pin 13 → L298N ENA
 * - ESP32 Pin 12 → L298N IN1
 * - ESP32 Pin 14 → L298N IN2  
 * - ESP32 GND → L298N GND (CRITICAL!)
 * - 12V Power (+) → L298N +12V
 * - 12V Power (-) → L298N GND
 * - Motor wires → OUT1, OUT2
 */

 #define PIN_ENA   21  // PWM speed control
 #define PIN_IN1   22  // Direction bit 1
 #define PIN_IN2   23  // Direction bit 2
 
 void setup() {
   Serial.begin(115200);
   Serial.println("=================================");
   Serial.println("DC Motor Test (L298N)");
   Serial.println("=================================");
   
   pinMode(PIN_ENA, OUTPUT);
   pinMode(PIN_IN1, OUTPUT);
   pinMode(PIN_IN2, OUTPUT);
   
   // Stop motor initially
   analogWrite(PIN_ENA, 0);
   
   Serial.println("Setup complete. Starting test...");
   delay(2000);
 }
 
 void loop() {
   // ====== FORWARD (SLOW) ======
   Serial.println("Forward at 100% speed...");
   digitalWrite(PIN_IN1, HIGH);
   digitalWrite(PIN_IN2, LOW);
   analogWrite(PIN_ENA, 255);  // 80/255 = ~30%
   delay(500);
   
   // ====== STOP ======
   Serial.println("STOP");
   analogWrite(PIN_ENA, 0);
   delay(1000);
   
   // ====== BACKWARD (SLOW) ======
   Serial.println("Backward at 100% speed...");
   digitalWrite(PIN_IN1, LOW);
   digitalWrite(PIN_IN2, HIGH);
   analogWrite(PIN_ENA, 255);
   delay(500);
   
   // ====== STOP ======
   Serial.println("STOP");
   analogWrite(PIN_ENA, 0);
   delay(2000);
 }
 
 /*
  * TROUBLESHOOTING:
  * 
  * Motor doesn't move at all:
  * 1. Check L298N power LED (should be ON)
  * 2. Verify ESP32 GND connected to L298N GND (use multimeter!)
  * 3. Check 12V connected to L298N +12V terminal
  * 4. Try removing ENA jumper on L298N
  * 5. Try digitalWrite(PIN_ENA, HIGH) instead of analogWrite
  * 6. Swap motor wires (OUT1 ↔ OUT2)
  * 
  * Motor spins but very weak:
  * - Increase analogWrite value (try 150-200)
  * - Check motor voltage rating (might need more than 12V)
  * 
  * Motor spins opposite direction:
  * - Flip IN1 and IN2 values
  * - OR swap motor wires physically
  */