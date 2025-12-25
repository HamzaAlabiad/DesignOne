#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ===== Must match Master =====
#define ESPNOW_CHANNEL 1

// Onboard LED (change if your board uses a different pin)
#define PIN_LED 2

uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ===== Must match Master's protocol =====
enum MsgType : uint8_t { MSG_HELLO = 1, MSG_PRODUCT = 2, MSG_RESET = 3 };

// Slave -> Master: discovery
struct __attribute__((packed)) HelloMsg {
  uint8_t type;     // MSG_HELLO
  uint8_t mac[6];   // Slave MAC
  uint32_t uptime;  // millis()
};

// Master -> Slave: product classification
struct __attribute__((packed)) ProductMsg {
  uint8_t type;   // MSG_PRODUCT
  uint8_t seq;    // product sequence number
  uint8_t color;  // unused in this test
  uint8_t part;   // unused in this test
};

// Slave -> Master: reset/unlock next cycle
struct __attribute__((packed)) ResetMsg {
  uint8_t type;     // MSG_RESET
  uint8_t seqDone;  // which product finished
};

// ===== State =====
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

void blinkFast(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(80);
    digitalWrite(PIN_LED, LOW);
    delay(80);
  }
}

// New ESP32 Arduino core callback signature
void onRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len < 1) return;

  uint8_t t = data[0];

  // Master sends PRODUCT -> we reply RESET
  if (t == MSG_PRODUCT && len >= (int)sizeof(ProductMsg)) {
    ProductMsg p;
    memcpy(&p, data, sizeof(p));

    // Learn Master's MAC from the received packet
    if (!haveMaster) {
      memcpy(masterMac, recv_info->src_addr, 6);
      haveMaster = true;
      addPeer(masterMac);
    }

    // Visual feedback: received a product
    blinkFast(2);

    // Reply with RESET to unlock Master
    ResetMsg r;
    r.type = MSG_RESET;
    r.seqDone = p.seq;

    esp_now_send(masterMac, (uint8_t *)&r, sizeof(r));

    // Visual feedback: sent reset
    blinkFast(1);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    // Blink forever if ESPNOW init fails
    while (true) {
      digitalWrite(PIN_LED, HIGH);
      delay(300);
      digitalWrite(PIN_LED, LOW);
      delay(300);
    }
  }

  esp_now_register_recv_cb(onRecv);

  // Add broadcast peer (helps on some stacks)
  addPeer(BROADCAST_MAC);
}

void loop() {
  // Broadcast HELLO every 500ms so Master can discover us
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

  // Idle heartbeat blink (slow) while waiting
  digitalWrite(PIN_LED, HIGH);
  delay(30);
  digitalWrite(PIN_LED, LOW);
  delay(970);
}