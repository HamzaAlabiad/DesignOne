/*
 * Test: 2-Channel Relay Module
 * Expected: Relays click ON/OFF, controlled devices activate
 * 
 * WIRING:
 * - Relay VCC → 5V
 * - Relay GND → GND
 * - Relay IN1 → GPIO 25 (Feeder)
 * - Relay IN2 → GPIO 26 (Vacuum)
 * 
 * RELAY OUTPUTS:
 * - Relay 1 COM → Solenoid valve (+)
 * - Relay 1 NO → 12V (+)
 * - Relay 2 COM → Vacuum pump (+)
 * - Relay 2 NO → 12V (+)
 */

 #define PIN_RELAY_FEEDER  26
 #define PIN_RELAY_VACUUM  25
 
 void setup() {
   Serial.begin(115200);
   Serial.println("=================================");
   Serial.println("Relay Test");
   Serial.println("=================================");
   
   pinMode(PIN_RELAY_FEEDER, OUTPUT);
   pinMode(PIN_RELAY_VACUUM, OUTPUT);
   
   // Turn off both relays initially
   digitalWrite(PIN_RELAY_FEEDER, HIGH);
   digitalWrite(PIN_RELAY_VACUUM, HIGH);
   
   Serial.println("Starting relay test...");
   delay(2000);
 }
 
 void loop() {
   // ====== TEST RELAY 1 (FEEDER) ======
   Serial.println("Relay 1 (Feeder): ON");
   digitalWrite(PIN_RELAY_FEEDER, LOW);
   delay(2000);
   
   Serial.println("Relay 1 (Feeder): OFF");
   digitalWrite(PIN_RELAY_FEEDER, HIGH);
   delay(2000);
   
   // ====== TEST RELAY 2 (VACUUM) ======
   Serial.println("Relay 2 (Vacuum): ON");
   digitalWrite(PIN_RELAY_VACUUM, LOW);
   delay(2000);
   
   Serial.println("Relay 2 (Vacuum): OFF");
   digitalWrite(PIN_RELAY_VACUUM, HIGH);
   delay(2000);
 }
 
 /*
  * EXPECTED BEHAVIOR:
  * - You should HEAR a click when relay activates
  * - Relay LED should light up
  * - Connected device (solenoid/vacuum) should activate
  * 
  * TROUBLESHOOTING:
  * 
  * Relay doesn't click:
  * - Check 5V power to relay module
  * - Check GND connection
  * - Try inverting signal (Low ↔ LOW) - some relays are active LOW
  * 
  * Relay clicks but device doesn't work:
  * - Check device power supply (12V connected?)
  * - Verify COM and NO wiring
  * - Test device directly with 12V (bypass relay)
  * - Check device fuse/protection
  * 
  * Relay clicks but ESP32 resets:
  * - Relay draws too much current
  * - Add external transistor driver circuit
  * - Use separate power supply for relay logic
  */