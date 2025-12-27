/*
 * ESP32-A MASTER - FreeRTOS CONVERSION
 * ACTIVITY-BASED ARCHITECTURE
 * Modified feeding logic preserved:
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
  MSG_VACUUM_ON = 6
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
  uint8_t state;
  uint32_t timestamp;
};

// ============================================================================
// RTOS SYNCHRONIZATION PRIMITIVES
// ============================================================================
SemaphoreHandle_t xLCDMutex;
SemaphoreHandle_t xVacuumSignalSemaphore;
SemaphoreHandle_t xMotionCompleteSemaphore;
SemaphoreHandle_t xPairingMutex;

TaskHandle_t xFeederTaskHandle = NULL;

// ============================================================================
// GLOBAL STATE (Protected by mutexes where needed)
// ============================================================================
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

bool paired = false;
uint8_t slaveMac[6] = {0};

String lastL1 = "";
String lastL2 = "";

uint32_t productCount = 0;

// ============================================================================
// LCD HELPER (Mutex-protected)
// ============================================================================
void lcdShow(const String &l1, const String &l2) {
  String a = l1, b = l2;
  if (a.length() > 16) a = a.substring(0, 16);
  if (b.length() > 16) b = b.substring(0, 16);
  while (a.length() < 16) a += " ";
  while (b.length() < 16) b += " ";
  
  if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE) {
    if (a == lastL1 && b == lastL2) {
      xSemaphoreGive(xLCDMutex);
      return;
    }
    lastL1 = a;
    lastL2 = b;
    lcd.setCursor(0, 0);
    lcd.print(a);
    lcd.setCursor(0, 1);
    lcd.print(b);
    xSemaphoreGive(xLCDMutex);
  }
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
    if (xSemaphoreTake(xPairingMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (!paired) {
        memcpy(slaveMac, msg.mac, 6);
        paired = true;
        addPeer(slaveMac);
      }
      xSemaphoreGive(xPairingMutex);
    }
    return;
  }
  
  if (t == MSG_MOTION_COMPLETE && len >= (int)sizeof(MotionCompleteMsg)) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xMotionCompleteSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return;
  }
  
  if (t == MSG_VACUUM_ON && len >= (int)sizeof(VacuumMsg)) {
    VacuumMsg msg;
    memcpy(&msg, data, sizeof(msg));
    if (msg.state == 1) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(xVacuumSignalSemaphore, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    return;
  }
}

// ============================================================================
// SENSING FUNCTIONS
// ============================================================================
uint8_t readColor() {
  int reading = digitalRead(PIN_IR_COLOR);
  vTaskDelay(pdMS_TO_TICKS(50));
  reading = digitalRead(PIN_IR_COLOR);
  
  if (reading == LOW) {
    return 1;  // WHITE
  } else {
    return 0;  // BLACK
  }
}

uint8_t readType() {
  int reading = digitalRead(PIN_LASER_RX);
  vTaskDelay(pdMS_TO_TICKS(50));
  reading = digitalRead(PIN_LASER_RX);
  
  if (reading == HIGH) {
    return 0;  // BASE (tall)
  } else {
    return 1;  // LID (short)
  }
}

// ============================================================================
// FEEDER TASK - MAIN PROCESS LOGIC
// ============================================================================
void vFeederTask(void *pvParameters) {
  (void)pvParameters;
  
  for (;;) {
    lcdShow("READY", "Place product...");
    
    // 1. WAIT FOR PRODUCT AT FEEDER
    while (digitalRead(PIN_IR_FEEDER) == LOW) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    productCount++;
    
    // 2. FEEDING - Push until sorting position
    lcdShow("FEEDING", "Pushing...");
    
    digitalWrite(PIN_RELAY_FEEDER, LOW);  // Feeder ON
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wait for sorting position (limit switch pressed)
    TickType_t feedStart = xTaskGetTickCount();
    while (digitalRead(PIN_LIMIT_SORT) == LOW) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if ((xTaskGetTickCount() - feedStart) > pdMS_TO_TICKS(8000)) {
        digitalWrite(PIN_RELAY_FEEDER, HIGH);
        lcdShow("ERROR", "Feed timeout!");
        vTaskDelay(pdMS_TO_TICKS(3000));
        continue;  // Restart process
      }
    }
    
    // *** MODIFIED: Keep feeder extended, wait for vacuum signal ***
    lcdShow("FEEDING", "Product ready");
    // Feeder stays ON (extended)
    
    // 3. SORTING - Triggered by limit switch
    vTaskDelay(pdMS_TO_TICKS(500));  // Stabilize
    
    lcdShow("SORTING", "Reading...");
    
    uint8_t color = readColor();
    vTaskDelay(pdMS_TO_TICKS(200));
    uint8_t type = readType();
    
    // 4. DISPLAY RESULT
    String colorStr = color ? "WHT" : "BLK";
    String typeStr = type ? "LID" : "BASE";
    lcdShow(colorStr + " " + typeStr, "Sending...");
    
    // 5. SEND TO SLAVE
    bool isPaired = false;
    if (xSemaphoreTake(xPairingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      isPaired = paired;
      xSemaphoreGive(xPairingMutex);
    }
    
    if (!isPaired) {
      digitalWrite(PIN_RELAY_FEEDER, HIGH);
      lcdShow("ERROR", "No slave!");
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
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
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }
    
    // 6. WAIT FOR ROBOT TO START PICKING (vacuum signal)
    lcdShow("WAITING", "Robot moving...");
    
    // Clear semaphore before waiting
    xSemaphoreTake(xVacuumSignalSemaphore, 0);
    
    if (xSemaphoreTake(xVacuumSignalSemaphore, pdMS_TO_TICKS(30000)) != pdTRUE) {
      digitalWrite(PIN_RELAY_FEEDER, HIGH);
      lcdShow("ERROR", "Vacuum timeout!");
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }
    
    // *** NEW: Vacuum is ON, now retract feeder ***
    lcdShow("RETRACTING", "Feeder...");
    digitalWrite(PIN_RELAY_FEEDER, HIGH);  // Feeder OFF (retract)
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 7. WAIT FOR MOTION COMPLETE
    lcdShow("WAITING", "Robot placing...");
    
    // Clear semaphore before waiting
    xSemaphoreTake(xMotionCompleteSemaphore, 0);
    
    if (xSemaphoreTake(xMotionCompleteSemaphore, pdMS_TO_TICKS(45000)) != pdTRUE) {
      lcdShow("ERROR", "Motion timeout!");
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }
    
    lcdShow("COMPLETE!", "Count: " + String(productCount));
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // Match original delay before next cycle
  }
}

// ============================================================================
// INITIALIZATION TASK - Runs once then deletes itself
// ============================================================================
void vInitTask(void *pvParameters) {
  (void)pvParameters;
  
  // INIT LCD
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdShow("ESP32-A MASTER", "Starting...");
  vTaskDelay(pdMS_TO_TICKS(2000));
  
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
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(PIN_RELAY_FEEDER, HIGH);
  vTaskDelay(pdMS_TO_TICKS(500));
  
  lcdShow("INIT", "ESP-NOW...");
  
  // INIT ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    lcdShow("ERROR", "ESP-NOW failed!");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);
  addPeer(BROADCAST_MAC);
  
  uint8_t myMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
  
  lcdShow("MASTER MAC", macToShortStr(myMac));
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  // WAIT FOR SLAVE
  lcdShow("WAITING SLAVE", "Please wait...");
  
  TickType_t pairStart = xTaskGetTickCount();
  bool isPaired = false;
  
  while (!isPaired && ((xTaskGetTickCount() - pairStart) < pdMS_TO_TICKS(15000))) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (xSemaphoreTake(xPairingMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      isPaired = paired;
      xSemaphoreGive(xPairingMutex);
    }
  }
  
  if (isPaired) {
    lcdShow("SLAVE FOUND!", macToShortStr(slaveMac));
    vTaskDelay(pdMS_TO_TICKS(2000));
  } else {
    lcdShow("NO SLAVE", "Continuing...");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  
  lcdShow("SYSTEM READY!", "");
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  // Start feeder task
  xTaskCreatePinnedToCore(
    vFeederTask,
    "FeederTask",
    4096,
    NULL,
    2,
    &xFeederTaskHandle,
    1
  );
  
  // Delete initialization task
  vTaskDelete(NULL);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Create synchronization primitives
  xLCDMutex = xSemaphoreCreateMutex();
  xVacuumSignalSemaphore = xSemaphoreCreateBinary();
  xMotionCompleteSemaphore = xSemaphoreCreateBinary();
  xPairingMutex = xSemaphoreCreateMutex();
  
  if (xLCDMutex == NULL || xVacuumSignalSemaphore == NULL || 
      xMotionCompleteSemaphore == NULL || xPairingMutex == NULL) {
    while (true) {
      delay(1000);  // Fatal error - hang
    }
  }
  
  // Create initialization task
  xTaskCreatePinnedToCore(
    vInitTask,
    "InitTask",
    4096,
    NULL,
    3,
    NULL,
    1
  );
}

// ============================================================================
// LOOP - Empty (FreeRTOS handles everything)
// ============================================================================
void loop() {
  vTaskDelay(portMAX_DELAY);
}
