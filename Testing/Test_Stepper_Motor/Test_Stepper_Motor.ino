// Test stepper motor X-axis
void setup() {
  Serial.begin(115200);
  Serial.println("Testing Stepper X-Axis");
  
  pinMode(19, OUTPUT);  // STEP
  pinMode(18, OUTPUT);  // DIR
  pinMode(5, OUTPUT);   // ENABLE
  
  digitalWrite(5, LOW);  // Enable motor (active LOW!)
  
  Serial.println("Motor enabled. Starting movement...");
}

void loop() {
  // Move forward
  Serial.println("Moving forward 200 steps...");
  digitalWrite(18, HIGH);  
  for(int i = 0; i < 200; i++) {
    digitalWrite(19, HIGH);
    delayMicroseconds(2000);
    digitalWrite(19, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);
  
  // Move backward
  Serial.println("Moving backward 200 steps...");
  digitalWrite(18, LOW);
  for(int i = 0; i < 200; i++) {
    digitalWrite(19, HIGH);
    delayMicroseconds(2000);
    digitalWrite(19, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);
}