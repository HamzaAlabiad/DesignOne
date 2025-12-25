/*
 * ESP32 MASTER - INTEGRATED SIMPLE CODE
 * Feeding, Sorting, LCD, ESP-NOW
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_IR_FEEDER    19
#define PIN_IR_COLOR     23
#define PIN_RELAY_FEEDER 26
#define PIN_RELAY_VACUUM 25
#define PIN_LCD_SDA      21
#define PIN_LCD_SCL      22

// ============================================================================
// OBJECTS
// ============================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================================
// ESP-NOW
// ============================================================================
// REPLACE THIS WITH YOUR SLAVE'S MAC ADDRESS!
uint8_t slaveMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct ProductData {
  bool isBase;
  bool isWhite;
};

// ============================================================================
// STATE MACHINE
// ============================================================================
enum State {
  IDLE,
  FEEDING,
  WAITING_PICKUP
};

State currentState = IDLE;

// ============================================================================
// FUNCTIONS
// ============================================================================

void initRelays() {
  pinMode(PIN_RELAY_FEEDER, OUTPUT);
  pinMode(PIN_RELAY_VACUUM, OUTPUT);
  digitalWrite(PIN_RELAY_FEEDER, HIGH);  // OFF
  digitalWrite(PIN_RELAY_VACUUM, HIGH);  // OFF
}

void feederExtend() {
  digitalWrite(PIN_RELAY_FEEDER, LOW);
}

void feederRetract() {
  digitalWrite(PIN_RELAY_FEEDER, HIGH);
}

void vacuumOn() {
  digitalWrite(PIN_RELAY_VACUUM, LOW);
}

void vacuumOff() {
  digitalWrite(PIN_RELAY_VACUUM, HIGH);
}

void initSensors() {
  pinMode(PIN_IR_FEEDER, INPUT);
  pinMode(PIN_IR_COLOR, INPUT);
}

bool objectAtFeeder() {
  return digitalRead(PIN_IR_FEEDER) == HIGH;
}

bool isWhiteObject() {
  return digitalRead(PIN_IR_COLOR) == LOW;  // LOW = white
}

void initLCD() {
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void showMessage(String line1, String line2) {
  while (line1.length() < 16) line1 += " ";
  while (line2.length() < 16) line2 += " ";
  
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void initESPNOW() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    showMessage("ESP-NOW ERROR", "");
    while(1);
  }
  
  // Add slave peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, slaveMac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);
}

void sendProductToSlave(bool isBase, bool isWhite) {
  ProductData data;
  data.isBase = isBase;
  data.isWhite = isWhite;
  
  esp_err_t result = esp_now_send(slaveMac, (uint8_t*)&data, sizeof(data));
  
  if (result == ESP_OK) {
    showMessage("Sent to Robot", isBase ? "Base" : "Lid");
  } else {
    showMessage("Send FAILED", "");
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  
  initRelays();
  initSensors();
  initLCD();
  initESPNOW();
  
  // Print MAC address
  uint8_t myMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
  Serial.print("Master MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", myMac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  showMessage("System Ready", "");
  delay(2000);
  showMessage("Waiting...", "");
  
  currentState = IDLE;
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  switch (currentState) {
    
    case IDLE:
      // Check if object at feeder
      if (objectAtFeeder()) {
        showMessage("Feeding...", "");
        feederExtend();
        delay(2000);  // Wait for cylinder to extend
        
        currentState = FEEDING;
      }
      break;
    
    case FEEDING:
      // Wait a bit, then classify
      delay(1000);  // Allow product to settle
      
      // Read sensors
      bool isWhite = isWhiteObject();
      
      // For now, assume everything is a BASE
      // In real system, you'd have a laser sensor to detect base vs lid
      bool isBase = true;  // TODO: Add laser sensor reading
      
      // Display classification
      showMessage(isBase ? "Base" : "Lid", 
                  isWhite ? "White" : "Black");
      
      // Send to slave
      sendProductToSlave(isBase, isWhite);
      
      currentState = WAITING_PICKUP;
      break;
    
    case WAITING_PICKUP:
      // Wait for object to be removed (simulate pickup)
      delay(5000);  // Wait 5 seconds (robot picks up)
      
      // Retract feeder
      feederRetract();
      
      showMessage("Waiting...", "");
      currentState = IDLE;
      break;
  }
  
  delay(100);
}