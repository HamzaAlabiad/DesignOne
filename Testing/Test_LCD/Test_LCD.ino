#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PIN_LCD_SDA 21
#define PIN_LCD_SCL 22

// Set this to the address found by the scanner:
#define LCD_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

void setup() {
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);

  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCD TEST");
  lcd.setCursor(0, 1);
  lcd.print("ESP32-A OK");
}

void loop() {
  // Update a small counter on row 1 so you know it's alive
  lcd.setCursor(11, 1);
  lcd.print(millis() / 1000);
  lcd.print("  "); // clear leftover digits

  delay(500);
}