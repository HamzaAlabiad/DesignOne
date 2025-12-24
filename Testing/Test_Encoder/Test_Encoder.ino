#include <ESP32Encoder.h>
#define encoderPinA 19  // Replace with your actual pins
#define encoderPinB 18 
ESP32Encoder encoder;
void setup() {
  Serial.begin(115200);
  encoder.attachFullQuad(encoderPinA, encoderPinB);
  encoder.setCount(0); // Start at zero
  Serial.println("Move the motor a known distance, then check the count.");
}
void loop() {
  // Print the count every 500ms so you can see it change
  Serial.print("Current Encoder Count: ");
  Serial.println((long)encoder.getCount());
  delay(500);
}