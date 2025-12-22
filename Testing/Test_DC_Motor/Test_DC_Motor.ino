#define ENA_PIN  4
#define IN1_PIN  13
#define IN2_PIN  14

void setup() {
  Serial.begin(115200);
  Serial.println("Testing DC Motor Z-Axis");
  
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
}

void loop() {
  // Forward SLOW
  Serial.println("Forward at 30% speed");
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 80);  // Start slow!
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(ENA_PIN, 0);
  delay(2000);
  
  // Backward SLOW
  Serial.println("Backward at 30% speed");
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  analogWrite(ENA_PIN, 80);
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(ENA_PIN, 0);
  delay(2000);
}