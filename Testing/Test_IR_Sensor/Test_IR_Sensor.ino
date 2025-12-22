/*
 * Test: IR Sensors (Feeder position + Color detection)
 * Expected: Detects objects and differentiates black/white
 * 
 * WIRING:
 * - IR Sensor 1 OUT → GPIO 34 (Feeder)
 * - IR Sensor 2 OUT → GPIO 35 (Color)
 * - Both VCC → 3.3V or 5V
 * - Both GND → GND
 */

 #define PIN_IR_FEEDER  19
 #define PIN_IR_COLOR   23
 
 void setup() {
   Serial.begin(115200);
   Serial.println("=================================");
   Serial.println("IR Sensor Test");
   Serial.println("=================================");
   
   pinMode(PIN_IR_FEEDER, INPUT);
   pinMode(PIN_IR_COLOR, INPUT);
   
   Serial.println("Place objects in front of sensors...");
   delay(1000);
 }
 
 void loop() {
   int feeder = digitalRead(PIN_IR_FEEDER);
   int color = digitalRead(PIN_IR_COLOR);
   
   Serial.print("IR Feeder: ");
   Serial.print(feeder);
   Serial.print("  |  IR Color: ");
   Serial.println(color);
   
   // Interpretation
   if (feeder == HIGH) {
     Serial.println("  → Object detected at feeder");
   }
   
   if (color == LOW) {
     Serial.println("  → WHITE object detected");
   } else {
     Serial.println("  → BLACK object detected (or no object)");
   }
   
   Serial.println();
   delay(500);
 }
 
 /*
  * TEST PROCEDURE:
  * 1. Run code, open Serial Monitor
  * 2. Place WHITE paper in front of IR Color sensor
  *    - Should show: IR Color: 1 → WHITE
  * 3. Place BLACK paper in front
  *    - Should show: IR Color: 0 → BLACK
  * 4. Place any object near IR Feeder
  *    - Should show: IR Feeder: 1
  * 
  * TROUBLESHOOTING:
  * 
  * Always reads 0:
  * - Check sensor power (some have LEDs)
  * - Verify VCC connected to 3.3V or 5V
  * - Check GND connected
  * - Some sensors are active LOW (invert reading)
  * 
  * Always reads 1:
  * - Pin might be floating (add 10kΩ pulldown resistor)
  * - Check sensor type (might need analog read)
  * 
  * Can't differentiate black/white:
  * - Sensor might be digital only (reads object/no object)
  * - Adjust sensor distance (closer = better)
  * - Check sensor has adjustable sensitivity (potentiometer)
  */