/*
 * ESP32-A MASTER - LCD ONLY VERSION
 * Modified feeding logic:
 * - Sorting triggers when limit switch pressed
 * - Feeder retracts ONLY when vacuum turns on during picking
 */

 #include <WiFi.h>
 #include <esp_now.h>
 #include <esp_wifi.h>
 #include <Wire.h>
 #include <LiquidCrystal_I2C.h>
 
 // ============================================================================
 // PIN DEFINITIONS
 // ============================================================================
 #define PIN_LCD_SDA 21
 #define PIN_LCD_SCL 22
 #define LCD_ADDR 0x27
 
 #define PIN_IR_FEEDER  19
 #define PIN_IR_COLOR   23
 #define PIN_LASER_RX   32
 #define PIN_LIMIT_SORT 33
 
 #define PIN_RELAY_FEEDER  26
 
 // ============================================================================
 // ESP-NOW
 // ============================================================================
 #define ESPNOW_CHANNEL 1
 uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 
 enum MsgType : uint8_t {
   MSG_HELLO = 1,
   MSG_PING = 2,
   MSG_ACK = 3,
   MSG_PRODUCT_DATA = 4,
   MSG_MOTION_COMPLETE = 5,
   MSG_VACUUM_ON = 6  // NEW: Signal from slave when vacuum activates
 };
 
 struct __attribute__((packed)) HelloMsg {
   uint8_t type;
   uint8_t mac[6];
   uint32_t uptime;
 };
 
 struct __attribute__((packed)) ProductMsg {
   uint8_t type;
   uint8_t color;
   uint8_t productType;
   uint32_t timestamp;
 };
 
 struct __attribute__((packed)) MotionCompleteMsg {
   uint8_t type;
   uint8_t success;
   uint32_t timestamp;
 };
 
 struct __attribute__((packed)) VacuumMsg {
   uint8_t type;
   uint8_t state;  // 1 = ON, 0 = OFF
   uint32_t timestamp;
 };
 
 // ============================================================================
 // GLOBAL STATE
 // ============================================================================
 LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
 
 bool paired = false;
 uint8_t slaveMac[6] = {0};
 volatile bool gotMotionComplete = false;
 volatile bool gotVacuumOn = false;
 
 String lastL1 = "";
 String lastL2 = "";
 
 uint32_t productCount = 0;
 
 // ============================================================================
 // LCD HELPER
 // ============================================================================
 void lcdShow(const String &l1, const String &l2) {
   String a = l1, b = l2;
   if (a.length() > 16) a = a.substring(0, 16);
   if (b.length() > 16) b = b.substring(0, 16);
   while (a.length() < 16) a += " ";
   while (b.length() < 16) b += " ";
   if (a == lastL1 && b == lastL2) return;
   lastL1 = a;
   lastL2 = b;
   lcd.setCursor(0, 0);
   lcd.print(a);
   lcd.setCursor(0, 1);
   lcd.print(b);
 }
 
 String macToShortStr(const uint8_t *mac) {
   char buf[9];
   snprintf(buf, sizeof(buf), "%02X:%02X:%02X", mac[3], mac[4], mac[5]);
   return String(buf);
 }
 
 // ============================================================================
 // ESP-NOW FUNCTIONS
 // ============================================================================
 void addPeer(const uint8_t *mac) {
   esp_now_peer_info_t peer = {};
   memcpy(peer.peer_addr, mac, 6);
   peer.channel = ESPNOW_CHANNEL;
   peer.encrypt = false;
   peer.ifidx = WIFI_IF_STA;
   esp_now_add_peer(&peer);
 }
 
 void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
   (void)tx_info;
 }
 
 void onRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
   if (len < 1) return;
   uint8_t t = data[0];
   
   if (t == MSG_HELLO && len >= (int)sizeof(HelloMsg)) {
     HelloMsg msg;
     memcpy(&msg, data, sizeof(msg));
     if (!paired) {
       memcpy(slaveMac, msg.mac, 6);
       paired = true;
       addPeer(slaveMac);
     }
     return;
   }
   
   if (t == MSG_MOTION_COMPLETE && len >= (int)sizeof(MotionCompleteMsg)) {
     gotMotionComplete = true;
     return;
   }
   
   // NEW: Listen for vacuum activation signal
   if (t == MSG_VACUUM_ON && len >= (int)sizeof(VacuumMsg)) {
     VacuumMsg msg;
     memcpy(&msg, data, sizeof(msg));
     if (msg.state == 1) {
       gotVacuumOn = true;
     }
     return;
   }
 }
 
 // ============================================================================
 // SENSING FUNCTIONS
 // ============================================================================
 uint8_t readColor() {
   int reading = digitalRead(PIN_IR_COLOR);
   delay(50);
   reading = digitalRead(PIN_IR_COLOR);
   
   if (reading == LOW) {
     return 1;  // WHITE
   } else {
     return 0;  // BLACK
   }
 }
 
 uint8_t readType() {
   int reading = digitalRead(PIN_LASER_RX);
   delay(50);
   reading = digitalRead(PIN_LASER_RX);
   
   if (reading == HIGH) {
     return 0;  // BASE (tall)
   } else {
     return 1;  // LID (short)
   }
 }
 
 // ============================================================================
 // MAIN PROCESS
 // ============================================================================
 void processOneProduct() {
   lcdShow("READY", "Place product...");
   
   // 1. WAIT FOR PRODUCT AT FEEDER
   while (digitalRead(PIN_IR_FEEDER) == LOW) {
     delay(100);
   }
   
   productCount++;
   
   // 2. FEEDING - Push until sorting position
   lcdShow("FEEDING", "Pushing...");
   
   digitalWrite(PIN_RELAY_FEEDER, LOW);  // Feeder ON
   delay(100);
   
   // Wait for sorting position (limit switch pressed)
   unsigned long feedStart = millis();
   while (digitalRead(PIN_LIMIT_SORT) == LOW) {
     delay(10);
     if (millis() - feedStart > 8000) {
       digitalWrite(PIN_RELAY_FEEDER, HIGH);
       lcdShow("ERROR", "Feed timeout!");
       delay(3000);
       return;
     }
   }
   
   // *** MODIFIED: Keep feeder extended, wait for vacuum signal ***
   lcdShow("FEEDING", "Product ready");
   // Feeder stays ON (extended)
   
   // 3. SORTING - Triggered by limit switch
   delay(500);  // Stabilize
   
   lcdShow("SORTING", "Reading...");
   
   uint8_t color = readColor();
   delay(200);
   uint8_t type = readType();
   
   // 4. DISPLAY RESULT
   String colorStr = color ? "WHT" : "BLK";
   String typeStr = type ? "LID" : "BASE";
   lcdShow(colorStr + " " + typeStr, "Sending...");
   
   // 5. SEND TO SLAVE
   if (!paired) {
     digitalWrite(PIN_RELAY_FEEDER, HIGH);
     lcdShow("ERROR", "No slave!");
     delay(3000);
     return;
   }
   
   ProductMsg p;
   p.type = MSG_PRODUCT_DATA;
   p.color = color;
   p.productType = type;
   p.timestamp = millis();
   
   esp_err_t result = esp_now_send(slaveMac, (uint8_t *)&p, sizeof(p));
   
   if (result != ESP_OK) {
     digitalWrite(PIN_RELAY_FEEDER, HIGH);
     lcdShow("ERROR", "Send failed!");
     delay(3000);
     return;
   }
   
   // 6. WAIT FOR ROBOT TO START PICKING (vacuum signal)
   lcdShow("WAITING", "Robot moving...");
   
   gotVacuumOn = false;
   unsigned long vacuumWaitStart = millis();
   
   while (!gotVacuumOn) {
     delay(50);
     if (millis() - vacuumWaitStart > 30000) {
       digitalWrite(PIN_RELAY_FEEDER, HIGH);
       lcdShow("ERROR", "Vacuum timeout!");
       delay(3000);
       return;
     }
   }
   
   // *** NEW: Vacuum is ON, now retract feeder ***
   lcdShow("RETRACTING", "Feeder...");
   digitalWrite(PIN_RELAY_FEEDER, HIGH);  // Feeder OFF (retract)
   delay(500);
   
   // 7. WAIT FOR MOTION COMPLETE
   lcdShow("WAITING", "Robot placing...");
   
   gotMotionComplete = false;
   unsigned long waitStart = millis();
   
   while (!gotMotionComplete) {
     delay(100);
     if (millis() - waitStart > 45000) {
       lcdShow("ERROR", "Motion timeout!");
       delay(3000);
       return;
     }
   }
   
   lcdShow("COMPLETE!", "Count: " + String(productCount));
   delay(1500);
 }
 
 // ============================================================================
 // SETUP
 // ============================================================================
 void setup() {
   // No Serial.begin() - LCD only mode
   
   // INIT LCD
   Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
   lcd.init();
   lcd.backlight();
   lcd.clear();
   lcdShow("ESP32-A MASTER", "Starting...");
   delay(2000);
   
   // INIT PINS
   pinMode(PIN_IR_FEEDER, INPUT);
   pinMode(PIN_IR_COLOR, INPUT);
   pinMode(PIN_LASER_RX, INPUT);
   pinMode(PIN_LIMIT_SORT, INPUT_PULLUP);
   
   pinMode(PIN_RELAY_FEEDER, OUTPUT);
   digitalWrite(PIN_RELAY_FEEDER, HIGH);  // OFF initially
   
   lcdShow("TESTING", "Relay...");
   
   // Test relay
   digitalWrite(PIN_RELAY_FEEDER, LOW);
   delay(500);
   digitalWrite(PIN_RELAY_FEEDER, HIGH);
   delay(500);
   
   lcdShow("INIT", "ESP-NOW...");
   
   // INIT ESP-NOW
   WiFi.mode(WIFI_STA);
   esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
   
   if (esp_now_init() != ESP_OK) {
     lcdShow("ERROR", "ESP-NOW failed!");
     while (true) delay(1000);
   }
   
   esp_now_register_send_cb(onSent);
   esp_now_register_recv_cb(onRecv);
   addPeer(BROADCAST_MAC);
   
   uint8_t myMac[6];
   esp_wifi_get_mac(WIFI_IF_STA, myMac);
   
   lcdShow("MASTER MAC", macToShortStr(myMac));
   delay(2000);
   
   // WAIT FOR SLAVE
   lcdShow("WAITING SLAVE", "Please wait...");
   
   unsigned long pairStart = millis();
   while (!paired) {
     delay(100);
     if (millis() - pairStart > 15000) {
       break;
     }
   }
   
   if (paired) {
     lcdShow("SLAVE FOUND!", macToShortStr(slaveMac));
     delay(2000);
   } else {
     lcdShow("NO SLAVE", "Continuing...");
     delay(2000);
   }
   
   lcdShow("SYSTEM READY!", "");
   delay(2000);
 }
 
 // ============================================================================
 // LOOP
 // ============================================================================
 void loop() {
   processOneProduct();
   delay(1000);
 }