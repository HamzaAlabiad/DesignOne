#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// ============================================
// PIN DEFINITIONS (Master ESP32)
// ============================================
#define PIN_IR_DETECT         19    // IR sensor on slider (product detection)
#define PIN_DETECT_LIMIT      13    // Limit switch at sorting station
#define PIN_IR_SORT           23    // IR sensor for color detection
#define PIN_LASER_RECEIVER    32    // Laser receiver (Base vs Lid)
#define PIN_RELAY_FEEDER      26    // Relay controlling feeder cylinder
#define PIN_RELAY_SUCTION     25    // Relay for suction (not used yet)

// Aliases for compatibility with your code logic
#define PIN_FEEDER_IR         PIN_IR_DETECT
#define PIN_SORTING_LIMIT     PIN_DETECT_LIMIT
#define COLOR_IR_PIN          PIN_IR_SORT
#define LASER_RECEIVER_PIN    PIN_LASER_RECEIVER
#define PIN_RELAY_PNEUMATIC   PIN_RELAY_FEEDER
#define MAGAZINE_IR_PIN       PIN_IR_DETECT

// ============================================
// SENSOR LOGIC CONSTANTS
// ============================================
#define MAGAZINE_IR_THRESHOLD 500   // Analog threshold

// Color IR Logic
#define COLOR_BLACK           LOW   // LOW = black
#define COLOR_WHITE           HIGH  // HIGH = white

// Laser Logic
#define LASER_BEAM_BLOCKED    HIGH  // HIGH when blocked (Base)
#define LASER_BEAM_CLEAR      LOW   // LOW when clear (Lid)

// ============================================
// PRODUCT TYPES
// ============================================
enum ProductType {
  WHITE_BASE = 0,
  WHITE_LID = 1,
  BLACK_BASE = 2,
  BLACK_LID = 3,
  UNKNOWN = 4
};

// ============================================
// LCD CONFIGURATION
// ============================================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adjust address if needed (0x27 or 0x3F)

// ============================================
// RTOS OBJECTS
// ============================================
SemaphoreHandle_t xProductAtStation;  // Signal from Feeder to Sorting
SemaphoreHandle_t xSystemReady;       // Signal that system is ready
QueueHandle_t xLCDQueue;              // Queue for LCD updates

// Product info structure
typedef struct {
    String type;
    String color;
    int productCount;
} ProductInfo;

// Global product counter
volatile int totalProducts = 0;

// ============================================
// TASK PROTOTYPES
// ============================================
void taskFeeder(void *pv);
void taskSorting(void *pv);
void taskLCD(void *pv);

// ============================================
// SETUP - THIS IS WHERE YOU CREATE TASKS
// ============================================
void setup() {
    Serial.begin(115200);  // Keep for debugging
    delay(1000);
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   AUTOMATED SORTING LINE - MASTER    ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    // Initialize pins
    pinMode(PIN_IR_DETECT, INPUT);
    pinMode(PIN_DETECT_LIMIT, INPUT_PULLUP);
    pinMode(PIN_LASER_RECEIVER, INPUT);
    pinMode(PIN_IR_SORT, INPUT);
    pinMode(PIN_RELAY_FEEDER, OUTPUT);
    pinMode(PIN_RELAY_SUCTION, OUTPUT);
    
    // Start with relays OFF (active LOW, so HIGH = OFF)
    digitalWrite(PIN_RELAY_FEEDER, HIGH);
    digitalWrite(PIN_RELAY_SUCTION, HIGH);
    
    Serial.println("✓ Pins initialized");
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  AUTOMATED");
    lcd.setCursor(0, 1);
    lcd.print("SORTING LINE");
    delay(2000);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    delay(1000);
    
    Serial.println("✓ LCD initialized");
    
    // ========================================
    // CREATE RTOS OBJECTS
    // ========================================
    Serial.println("\n→ Creating RTOS objects...");
    
    xProductAtStation = xSemaphoreCreateBinary();
    xSystemReady = xSemaphoreCreateBinary();
    xLCDQueue = xQueueCreate(5, sizeof(ProductInfo));
    
    if (!xProductAtStation || !xSystemReady || !xLCDQueue) {
        Serial.println("✗ Failed to create RTOS objects!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RTOS ERROR!");
        while(1) delay(1000);
    }
    
    Serial.println("✓ RTOS objects created");
    
    // System starts ready
    xSemaphoreGive(xSystemReady);
    
    // ========================================
    // CREATE TASKS - YES, YOU NEED THESE!
    // ========================================
    Serial.println("\n→ Creating tasks...");
    
    // Task 1: Feeder (Priority 3 - High)
    xTaskCreatePinnedToCore(
        taskFeeder,           // Task function
        "FeederTask",         // Task name
        3072,                 // Stack size (bytes)
        NULL,                 // Parameters
        3,                    // Priority (3 = high)
        NULL,                 // Task handle (not needed)
        0                     // Core 0
    );
    Serial.println("  ✓ Feeder Task created");
    
    // Task 2: Sorting (Priority 2 - Medium)
    xTaskCreatePinnedToCore(
        taskSorting,
        "SortingTask",
        3072,
        NULL,
        2,
        NULL,
        0
    );
    Serial.println("  ✓ Sorting Task created");
    
    // Task 3: LCD (Priority 1 - Low)
    xTaskCreatePinnedToCore(
        taskLCD,
        "LCDTask",
        2048,
        NULL,
        1,
        NULL,
        1                     // Core 1 (separate from control logic)
    );
    Serial.println("  ✓ LCD Task created");
    
    // Display ready message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("Waiting...");
    
    Serial.println("\n✓ System Ready - Tasks Running\n");
}

void loop() {
    // Empty - all logic runs in RTOS tasks
}

// ============================================
// TASK 1: FEEDER CONTROL
// ============================================
void taskFeeder(void *pv) {
    Serial.println("[FEEDER TASK] Started - Monitoring for products...\n");
    
    bool lastIRState = HIGH;  // Track IR state for edge detection
    
    for (;;) {
        // Read current IR sensor state
        bool currentIRState = digitalRead(PIN_FEEDER_IR);
        
        // Detect falling edge (product arrives: HIGH → LOW)
        if (currentIRState == LOW && lastIRState == HIGH) {
            Serial.println("\n[FEEDER] 🔔 Product detected on slider!");
            
            // Update LCD
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Product Detect");
            
            // Check if system is ready (not busy processing)
            if (xSemaphoreTake(xSystemReady, 0) == pdTRUE) {
                Serial.println("[FEEDER] System ready - Starting feed cycle");
                
                // ========================================
                // EXTEND CYLINDER (Relay ON)
                // ========================================
                Serial.println("[FEEDER] Activating relay (extending cylinder)...");
                
                lcd.setCursor(0, 1);
                lcd.print("Feeding...");
                
                digitalWrite(PIN_RELAY_PNEUMATIC, LOW);  // Relay ON (active LOW)
                
                Serial.println("[FEEDER] Cylinder extending → Product moving to station");
                
                // ========================================
                // WAIT FOR PRODUCT TO REACH STATION
                // ========================================
                Serial.println("[FEEDER] Waiting for limit switch trigger...");
                
                unsigned long feedStartTime = millis();
                bool limitHit = false;
                
                // Wait until limit switch is pressed (goes LOW)
                while (digitalRead(PIN_SORTING_LIMIT) == HIGH) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    
                    // Timeout safety (3 seconds)
                    if (millis() - feedStartTime > 3000) {
                        Serial.println("[FEEDER] ⚠️ WARNING: Timeout waiting for limit switch!");
                        Serial.println("[FEEDER] Product may be stuck or limit switch failed");
                        
                        lcd.clear();
                        lcd.setCursor(0, 0);
                        lcd.print("Feed Timeout!");
                        lcd.setCursor(0, 1);
                        lcd.print("Check System");
                        
                        break;
                    }
                }
                
                // Check if limit was actually hit
                if (digitalRead(PIN_SORTING_LIMIT) == LOW) {
                    limitHit = true;
                    Serial.println("[FEEDER] ✓ Limit switch HIT - Product at sorting station");
                    
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Product Arrived");
                    lcd.setCursor(0, 1);
                    lcd.print("Classifying...");
                    
                } else {
                    Serial.println("[FEEDER] ✗ Limit switch NOT hit - Error condition");
                }
                
                // ========================================
                // NOTIFY SORTING TASK
                // ========================================
                if (limitHit) {
                    Serial.println("[FEEDER] Notifying Sorting Task...\n");
                    xSemaphoreGive(xProductAtStation);  // Wake up sorting task
                } else {
                    // Error: Give back system ready semaphore
                    Serial.println("[FEEDER] Error - Releasing system for retry\n");
                    digitalWrite(PIN_RELAY_PNEUMATIC, HIGH);  // Retract cylinder
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("System Ready");
                    lcd.setCursor(0, 1);
                    lcd.print("Waiting...");
                    
                    xSemaphoreGive(xSystemReady);
                }
                
            } else {
                Serial.println("[FEEDER] ⚠️ System busy - Product ignored");
                Serial.println("[FEEDER] (Another product is being processed)\n");
                
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("System Busy");
                lcd.setCursor(0, 1);
                lcd.print("Please Wait...");
                vTaskDelay(pdMS_TO_TICKS(1500));
                
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("System Ready");
                lcd.setCursor(0, 1);
                lcd.print("Waiting...");
            }
        }
        
        // Update last state for edge detection
        lastIRState = currentIRState;
        
        // Poll at 10Hz (100ms period)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================
// TASK 2: SORTING & CLASSIFICATION
// ============================================
void taskSorting(void *pv) {
    ProductInfo item;
    
    Serial.println("[SORTING TASK] Started");
    
    for (;;) {
        // Wait for notification that product has reached sorting station
        if (xSemaphoreTake(xProductAtStation, portMAX_DELAY)) {
            Serial.println("[SORTING] Product at station - identifying...");
            
            // Stabilization delay (let product settle)
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // ========================================
            // SENSOR READING (Matching Sensors.h logic)
            // ========================================
            
            // 1. Read Magazine IR (for presence confirmation if needed)
            int magazineIRRaw = analogRead(MAGAZINE_IR_PIN);
            bool productPresent = (magazineIRRaw > MAGAZINE_IR_THRESHOLD);
            
            // 2. Read Color IR
            bool isBlack = (digitalRead(COLOR_IR_PIN) == COLOR_BLACK);  // LOW = black
            bool isWhite = (digitalRead(COLOR_IR_PIN) == COLOR_WHITE);  // HIGH = white
            
            // 3. Read Laser Receiver (Base vs Lid)
            bool laserBlocked = (digitalRead(LASER_RECEIVER_PIN) == LASER_BEAM_BLOCKED);  // HIGH = blocked
            bool laserClear = (digitalRead(LASER_RECEIVER_PIN) == LASER_BEAM_CLEAR);      // LOW = clear
            
            // ========================================
            // CLASSIFICATION LOGIC (from classifyProduct())
            // ========================================
            ProductType detectedProduct = UNKNOWN;
            
            bool isBase = laserBlocked;  // Base blocks laser
            bool isLid = laserClear;     // Lid allows laser through
            
            if (isBase && isBlack) {
                detectedProduct = BLACK_BASE;
                item.type = "BASE";
                item.color = "BLACK";
            }
            else if (isBase && isWhite) {
                detectedProduct = WHITE_BASE;
                item.type = "BASE";
                item.color = "WHITE";
            }
            else if (isLid && isBlack) {
                detectedProduct = BLACK_LID;
                item.type = "LID";
                item.color = "BLACK";
            }
            else if (isLid && isWhite) {
                detectedProduct = WHITE_LID;
                item.type = "LID";
                item.color = "WHITE";
            }
            else {
                detectedProduct = UNKNOWN;
                item.type = "UNKNOWN";
                item.color = "???";
            }
            
            // Increment product counter
            totalProducts++;
            item.productCount = totalProducts;
            
            // ========================================
            // DEBUG OUTPUT
            // ========================================
            Serial.println("\n========== CLASSIFICATION ==========");
            Serial.print("Magazine IR Raw: ");
            Serial.println(magazineIRRaw);
            Serial.print("Color IR: ");
            Serial.print(digitalRead(COLOR_IR_PIN));
            Serial.print(" → ");
            Serial.println(isBlack ? "BLACK" : "WHITE");
            Serial.print("Laser Receiver: ");
            Serial.print(digitalRead(LASER_RECEIVER_PIN));
            Serial.print(" → ");
            Serial.println(laserBlocked ? "BLOCKED (Base)" : "CLEAR (Lid)");
            Serial.print("\n>>> RESULT: ");
            Serial.print(item.color);
            Serial.print(" ");
            Serial.println(item.type);
            Serial.println("====================================\n");
            
            // ========================================
            // UPDATE LCD
            // ========================================
            xQueueSend(xLCDQueue, &item, 0);
            
            Serial.printf("[SORTING] Classification complete: %s %s\n", 
                         item.color.c_str(), item.type.c_str());
            
            // Wait to display classification result
            vTaskDelay(pdMS_TO_TICKS(2500));
            
            // ========================================
            // RETRACT FEEDER CYLINDER
            // ========================================
            Serial.println("[SORTING] Retracting feeder...");
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Retracting...");
            
            digitalWrite(PIN_RELAY_PNEUMATIC, HIGH);  // Retract (Relay OFF)
            
            // Wait for cylinder to fully retract (limit switch clears)
            Serial.println("[SORTING] Waiting for limit switch to clear...");
            unsigned long retractStartTime = millis();
            
            while (digitalRead(PIN_SORTING_LIMIT) == LOW) {  // Wait until HIGH (not pressed)
                vTaskDelay(pdMS_TO_TICKS(50));
                
                // Timeout safety (5 seconds)
                if (millis() - retractStartTime > 5000) {
                    Serial.println("[SORTING] ⚠️ WARNING: Retract timeout!");
                    
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Retract Timeout!");
                    lcd.setCursor(0, 1);
                    lcd.print("Check Cylinder");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    
                    break;
                }
            }
            
            Serial.println("[SORTING] ✓ Station cleared");
            
            // ========================================
            // COOLING PERIOD & SYSTEM READY
            // ========================================
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Ready for Next");
            lcd.setCursor(0, 1);
            lcd.print("Total: ");
            lcd.print(totalProducts);
            
            vTaskDelay(pdMS_TO_TICKS(1500));  // Stabilization delay
            
            Serial.println("[SORTING] System ready for next product\n");
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("System Ready");
            lcd.setCursor(0, 1);
            lcd.print("Waiting...");
            
            xSemaphoreGive(xSystemReady);  // Signal that system is ready
        }
    }
}

// ============================================
// TASK 3: LCD DISPLAY
// ============================================
void taskLCD(void *pv) {
    ProductInfo display;
    
    Serial.println("[LCD TASK] Started");
    
    for (;;) {
        // Wait for classification data from sorting task
        if (xQueueReceive(xLCDQueue, &display, portMAX_DELAY)) {
            
            Serial.println("[LCD] Updating display with classification result");
            
            // Display classification result
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(display.color);
            lcd.print(" ");
            lcd.print(display.type);
            
            lcd.setCursor(0, 1);
            lcd.print("Count: ");
            lcd.print(display.productCount);
            
            Serial.printf("[LCD] Displayed: %s %s (Count: %d)\n", 
                         display.color.c_str(), 
                         display.type.c_str(), 
                         display.productCount);
            
            // Keep this display showing
            // (Sorting task will update LCD after retraction)
        }
    }
}
