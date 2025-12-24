#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 1
#define PIN_LED 2

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

bool haveMaster = false;
uint8_t masterMac[6] = {0};

unsigned long lastHelloMs = 0;

void addPeer(const uint8_t *mac) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);
}

void blinkFast() {
  digitalWrite(PIN_LED, HIGH);
  delay(40);
  digitalWrite(PIN_LED, LOW);
}

// NEW signature (ESP32 Arduino 3.x / IDF5)
void onRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len < 1) return;

  uint8_t t = data[0];

  if (t == MSG_PING && len >= (int)sizeof(PingMsg)) {
    PingMsg p;
    memcpy(&p, data, sizeof(p));

    if (!haveMaster) {
      memcpy(masterMac, recv_info->src_addr, 6);
      haveMaster = true;
      addPeer(masterMac);
    }

    AckMsg a;
    a.type = MSG_ACK;
    a.seq = p.seq;
    a.uptime = millis();

    esp_now_send(masterMac, (uint8_t *)&a, sizeof(a));
    blinkFast();
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (true) {
      digitalWrite(PIN_LED, HIGH);
      delay(300);
      digitalWrite(PIN_LED, LOW);
      delay(300);
    }
  }

  esp_now_register_recv_cb(onRecv);
  addPeer(BROADCAST_MAC);
}

void loop() {
  unsigned long now = millis();
  if (now - lastHelloMs >= 500) {
    lastHelloMs = now;

    uint8_t myMac[6];
    esp_wifi_get_mac(WIFI_IF_STA, myMac);

    HelloMsg h;
    h.type = MSG_HELLO;
    memcpy(h.mac, myMac, 6);
    h.uptime = millis();

    esp_now_send(BROADCAST_MAC, (uint8_t *)&h, sizeof(h));
  }

  if (!haveMaster) {
    digitalWrite(PIN_LED, HIGH);
    delay(50);
    digitalWrite(PIN_LED, LOW);
    delay(950);
  } else {
    delay(50);
  }
}