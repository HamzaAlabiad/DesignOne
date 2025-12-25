#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
// --- Pin Definitions ---
#define PIN_FEEDER_IR 34      // Sensor at the very beginning
#define PIN_SORTING_LIMIT 35  // Limit switch at the end of piston stroke
#define PIN_COLOR_IR 32       // Analog IR for color
#define PIN_LASER_REC 33      // Laser digital receiver
#define PIN_RELAY_PNEUMATIC 25 // Controls the relay for the solenoid

// Feeder timing (ms)
#define FEED_PREPARE_MS 300 // wait before activating feeder to allow settling
#define FEED_PUSH_MIN_MS 150 // minimum actuator ON time to avoid overly-quick pushes





// --- RTOS Objects ---
SemaphoreHandle_t xProductAtStation; // Signal from Feeder to Sorting
SemaphoreHandle_t xSystemReady;      // Signal that a cycle is finished
QueueHandle_t xLCDQueue;
typedef struct {
    String type;
    String color;
} ProductInfo;
LiquidCrystal_I2C lcd(0x27, 16, 2);






// --- Task Prototypes ---
void taskFeeder(void *pv);
void taskSorting(void *pv);
void taskLCD(void *pv);
void setup() {
    Serial.begin(115200);
    pinMode(PIN_FEEDER_IR, INPUT);
    pinMode(PIN_SORTING_LIMIT, INPUT_PULLUP);
    pinMode(PIN_LASER_REC, INPUT);
    pinMode(PIN_RELAY_PNEUMATIC, OUTPUT);
    digitalWrite(PIN_RELAY_PNEUMATIC, HIGH); 
    lcd.init();
    lcd.backlight();
    lcd.print("System Ready");
    // Initialize RTOS Objects
    xProductAtStation = xSemaphoreCreateBinary();
    xSystemReady = xSemaphoreCreateBinary();
    xLCDQueue = xQueueCreate(5, sizeof(ProductInfo));
    // System starts ready
    xSemaphoreGive(xSystemReady);
    // Task Creation
    xTaskCreate(taskFeeder, "FeederTask", 2048, NULL, 3, NULL);
    xTaskCreate(taskSorting, "SortingTask", 2048, NULL, 2, NULL);
    xTaskCreate(taskLCD, "LCDTask", 2048, NULL, 1, NULL);
}



void loop() {} // Empty
// --- 1. FEEDER TASK ---
// Responsibility: Watch for new product and push to station
void taskFeeder(void *pv) {
    // Initial delay to let sensors stabilize on boot
    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;) {
        // Wait for Product Detection (Active HIGH)
        if (digitalRead(PIN_FEEDER_IR) == HIGH) {
            // Debounce/confirm
            vTaskDelay(pdMS_TO_TICKS(50));
            if (digitalRead(PIN_FEEDER_IR) == HIGH) {
                if (xSemaphoreTake(xSystemReady, 0) == pdTRUE) {
                    Serial.println("[Feeder] Valid Product & System Ready. Preparing to push...");

                    // Show on LCD
                    ProductInfo prepMsg;
                    prepMsg.type = "Preparing...";
                    prepMsg.color = "";
                    xQueueSend(xLCDQueue, &prepMsg, 0);

                    // Small prepare delay so product can settle before pushing
                    vTaskDelay(pdMS_TO_TICKS(FEED_PREPARE_MS));

                    Serial.println("[Feeder] Pushing product...");
                    ProductInfo feedMsg;
                    feedMsg.type = "Feeding...";
                    feedMsg.color = "";
                    xQueueSend(xLCDQueue, &feedMsg, 0);

                    digitalWrite(PIN_RELAY_PNEUMATIC, HIGH); // Extend relay
                    // Hold a minimal push time to avoid overly-fast actuation
                    vTaskDelay(pdMS_TO_TICKS(FEED_PUSH_MIN_MS));

                    // Wait for it to arrive at the station (Limit Pressed = HIGH)
                    int timeout = 0;
                    while(digitalRead(PIN_SORTING_LIMIT) == LOW && timeout < 500) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                        timeout++;
                    }

                    if (timeout >= 500) {
                        Serial.println("[Feeder] Error: Product never reached station!");
                    }

                    Serial.println("[Feeder] Product at station. Notifying Sorting Task.");
                    xSemaphoreGive(xProductAtStation); // Wake up the Sorting Task

                    // Give sorter time and pace input
                    Serial.println("[Feeder] Pausing 2s for cycle reset...");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}




// --- 2. SORTING TASK ---
// Responsibility: Identify product and reset feeder
void taskSorting(void *pv) {
    ProductInfo item;
    for (;;) {
        // Wait for notification from the Feeder Task
        if (xSemaphoreTake(xProductAtStation, portMAX_DELAY)) {
            Serial.println("[Sorting] Identifying product...");
            vTaskDelay(pdMS_TO_TICKS(500)); // Stabilization delay

            // Read sensors
            int rawColor = analogRead(PIN_COLOR_IR); // keep analog path
            // If you have a digital color sensor, change to digitalRead(PIN_COLOR_IR)
            item.color = (rawColor < 1800) ? "WHITE" : "BLACK";
            // Laser: HIGH = BLOCKED (BASE), LOW = CLEAR (LID)
            item.type = (digitalRead(PIN_LASER_REC) == HIGH) ? "BASE" : "LID";

            // Update LCD
            xQueueSend(xLCDQueue, &item, 0);
            Serial.printf("[Sorting] Done: %s %s\n", item.color.c_str(), item.type.c_str());

            // Retract Feeder
            digitalWrite(PIN_RELAY_PNEUMATIC, LOW);
            Serial.println("[Sorting] Retracting piston...");
            // Wait for piston to clear the limit switch (PRESSED = HIGH -> then it will go LOW when cleared)
            while(digitalRead(PIN_SORTING_LIMIT) == HIGH) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            Serial.println("[Sorting] Station cleared. System ready for next.");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Cooling period
            xSemaphoreGive(xSystemReady); // Allow Feeder Task to run again
        }
    }
}





// --- 3. LCD TASK ---
// Responsibility: Update display without blocking mechanics
void taskLCD(void *pv) {
    ProductInfo display;
    for (;;) {
        if (xQueueReceive(xLCDQueue, &display, portMAX_DELAY)) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Type: " + display.type);
            lcd.setCursor(0, 1);
            lcd.print("Color: " + display.color);
        }
    }
}