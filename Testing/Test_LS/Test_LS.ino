/*
 * Test: All Limit Switches
 * Expected: Detects when each limit switch is pressed
 * 
 * WIRING:
 * - Limit X: GPIO 36, Common to pin, NO to GND
 * - Limit Y: GPIO 39, Common to pin, NO to GND
 * - Limit Z: GPIO 34, Common to pin, NO to GND
 * - Limit Sort: GPIO 33, Common to pin, NO to GND
 */

 #define PIN_LIMIT_X     36
 #define PIN_LIMIT_Y     39
 #define PIN_LIMIT_Z     34
 #define PIN_LIMIT_SORT  13
 
 void setup() {
   Serial.begin(115200);
   Serial.println("=================================");
   Serial.println("Limit Switch Test");
   Serial.println("=================================");
   
   pinMode(PIN_LIMIT_X, INPUT_PULLUP);
   pinMode(PIN_LIMIT_Y, INPUT_PULLUP);
   pinMode(PIN_LIMIT_Z, INPUT_PULLUP);
   pinMode(PIN_LIMIT_SORT, INPUT_PULLUP);
   
   Serial.println("Press each limit switch...");
   delay(1000);
 }
 
 void loop() {
   int x = digitalRead(PIN_LIMIT_X);
   int y = digitalRead(PIN_LIMIT_Y);
   int z = digitalRead(PIN_LIMIT_Z);
   int sort = digitalRead(PIN_LIMIT_SORT);
   
   Serial.print("X: ");
   Serial.print(x == HIGH ? "PRESSED" : "released");
   Serial.print("  |  Y: ");
   Serial.print(y == HIGH ? "PRESSED" : "released");
   Serial.print("  |  Z: ");
   Serial.print(z == HIGH ? "PRESSED" : "released");
   Serial.print("  |  Sort: ");
   Serial.println(sort == HIGH ? "PRESSED" : "released");
   
   delay(200);
 }
 
 /*
  * TEST PROCEDURE:
  * 1. Press X limit switch → Should show "X: PRESSED"
  * 2. Release → Should show "X: released"
  * 3. Repeat for Y, Z, and Sort switches
  * 
  * TROUBLESHOOTING:
  * 
  * Always shows PRESSED:
  * - Wiring backwards (swap Common and NO)
  * - Remove INPUT_PULLUP if using external resistor
  * 
  * Always shows released:
  * - Check GND connection
  * - Switch might be damaged
  * - Try pressing harder
  * 
  * Reading unstable:
  * - Add debouncing (check twice with delay)
  * - Clean switch contacts (oxidation)
  */