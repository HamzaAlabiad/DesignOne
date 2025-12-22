/*
 * Test: Laser Sensor (Height detection for Base vs Lid)
 * Expected: Detects when beam is blocked/unblocked
 * 
 * WIRING:
 * - Laser Receiver OUT → GPIO 32
 * - Receiver VCC → 5V
 * - Receiver GND → GND
 * - Transmitter VCC → 5V
 * - Transmitter GND → GND
 */

 #define PIN_LASER_RX  32

 void setup() {
   Serial.begin(115200);
   Serial.println("=================================");
   Serial.println("Laser Sensor Test");
   Serial.println("=================================");
   
   pinMode(PIN_LASER_RX, INPUT);
   
   Serial.println("Align transmitter and receiver...");
   delay(2000);
 }
 
 void loop() {
   int laser = digitalRead(PIN_LASER_RX);
   Serial.print("Laser: ");if (laser == HIGH) {
Serial.println("BLOCKED (Base detected)");
} else {
Serial.println("CLEAR (Lid or no object)");
}delay(300);
}/*

TEST PROCEDURE:


Align laser transmitter pointing at receiver




Should read: "CLEAR"




Block beam with your hand




Should read: "BLOCKED"



LOGIC:


Laser CLEAR (beam not blocked) = Lid (shorter)




Laser BLOCKED (beam blocked) = Base (taller)



TROUBLESHOOTING:

Always reads BLOCKED:


Transmitter and receiver not aligned




Move them closer together




Check transmitter is powered and emitting (red light visible)



Always reads CLEAR:


Receiver might be damaged




Check polarity of wires




Some receivers are active HIGH (invert logic)



Reading unstable (flickers):


Misalignment (use tape to fix position)




Ambient light interference (shield with tube)




Add debouncing in code (check same value twice)
*/