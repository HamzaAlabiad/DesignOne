#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PIN_LCD_SDA 21
#define PIN_LCD_SCL 22
#define LCD_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

#define ESPNOW_CHANNEL 1

uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum MsgType : uint8_t { MSG_HELLO = 1, MSG_PING = 2, MSG_ACK = 3 };

struct __attribute__((packed)) HelloMsg {
  uint8_t type;
  uint8_t mac[6];
  uint32_t uptime;
};

struct __attribute__((packed)) PingMsg {
  uint8_t type;
  uint16_t seq;
  uint32_t uptime;
};

struct __attribute__((packed)) AckMsg {
  uint8_t type;
  uint16_t seq;
  uint32_t uptime;
};

bool paired = false;
uint8_t slaveMac[6] = {0};

volatile bool lastSendDone = false;
volatile bool lastSendOk = false;

volatile bool gotAck = false;
volatile uint16_t lastAckSeq = 0;

uint16_t pingSeq = 0;
unsigned long lastPingMs = 0;

String lastL1 = "";
String lastL2 = "";

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

void addPeer(const uint8_t *mac) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);
}

// NEW signature (ESP32 Arduino 3.x / IDF5)
void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  lastSendDone = true;
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

// NEW signature (ESP32 Arduino 3.x / IDF5)
void onRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len < 1) return;

  const uint8_t *src = recv_info->src_addr;
  uint8_t t = data[0];

  if (t == MSG_HELLO && len >= (int)sizeof(HelloMsg)) {
    HelloMsg msg;
    memcpy(&msg, data, sizeof(msg));

    if (!paired) {
      memcpy(slaveMac, msg.mac, 6);
      paired = true;
      addPeer(slaveMac);
    }
    (void)src;
    return;
  }

  if (t == MSG_ACK && len >= (int)sizeof(AckMsg)) {
    AckMsg a;
    memcpy(&a, data, sizeof(a));
    gotAck = true;
    lastAckSeq = a.seq;
    (void)src;
    return;
  }
}

void setup() {
  delay(500);
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcdShow("ESPNOW MASTER", "Booting...");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    lcdShow("ESPNOW INIT", "FAILED");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  addPeer(BROADCAST_MAC);

  uint8_t myMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, myMac);

  lcdShow("MASTER MAC", macToShortStr(myMac));
  delay(1200);
  lcd.clear();

  lcdShow("WAITING SLAVE", "HELLO...");
}

void loop() {
  if (!paired) {
    lcdShow("WAITING SLAVE", "HELLO...");
    delay(100);
    return;
  }

  unsigned long now = millis();
  if (now - lastPingMs >= 500) {
    lastPingMs = now;

    PingMsg p;
    p.type = MSG_PING;
    p.seq = pingSeq++;
    p.uptime = millis();

    lastSendDone = false;
    lastSendOk = false;

    esp_now_send(slaveMac, (uint8_t *)&p, sizeof(p));
  }

  String l1 = "SLV " + macToShortStr(slaveMac);

  String l2 = "S:";
  l2 += String(pingSeq);
  l2 += " A:";
  l2 += String((int)lastAckSeq);
  l2 += lastSendOk ? " OK" : " FL";

  lcdShow(l1, l2);
  delay(50);
}