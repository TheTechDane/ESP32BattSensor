// ============================================================
//  ESPNow-Tester.ino
//  Xiao ESP32-C6  —  Monitor ESP-NOW traffic + ping every 60s
// ============================================================

#define LOGLEVEL LOGLEVEL_VERBOSE
#include "logger.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ── Config ───────────────────────────────────────────────────
#define PING_INTERVAL_MS  60000
#define ESPNOW_CHANNEL    11
// ─────────────────────────────────────────────────────────────

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
unsigned long lastPing = 0;

// ── Send callback (ESP32-C6 core 3.x signature) ──────────────

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    loglnI("Ping sent → OK");
  } else {
    loglnE("Ping sent → FAILED");
  }
}

// ── Receive callback ─────────────────────────────────────────

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);

  logI("From: ");   logI(macStr);
  logI("  RSSI: "); logI(info->rx_ctrl->rssi);
  logI("  Len: ");  loglnI(len);

  // Print payload — as text if printable, hex otherwise
  bool printable = true;
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x20 || data[i] > 0x7E) { printable = false; break; }
  }

  if (printable) {
    char buf[len + 1];
    memcpy(buf, data, len);
    buf[len] = '\0';
    logD("Payload: ");
    loglnD(buf);
  } else {
    logD("Payload (hex): ");
    for (int i = 0; i < len; i++) {
      char hex[4];
      snprintf(hex, sizeof(hex), "%02X ", data[i]);
      logD(hex);
    }
    loglnD("");
  }
}

// ── Ping helper ──────────────────────────────────────────────

void sendPing() {
  const char *msg = "Ping!";
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)msg, strlen(msg));
  if (result != ESP_OK) {
    logE("esp_now_send error: ");
    loglnE(esp_err_to_name(result));
  }
}

// ── Setup ────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  loglnI("=== ESP-NOW Monitor | Xiao ESP32-C6 ===");

  // WiFi must be STA before ESP-NOW init
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Force a fixed channel so sender and receiver always match
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Diagnostics — check these match on both boards
  logI("MAC address : "); loglnI(WiFi.macAddress().c_str());
  logI("WiFi channel: "); loglnI(WiFi.channel());

  // Initialise ESP-NOW
  if (esp_now_init() != ESP_OK) {
    loglnE("ESP-NOW init failed! Halting.");
    while (true) delay(1000);
  }
  loglnI("ESP-NOW initialised");

  // Register callbacks
  esp_now_register_recv_cb(onDataReceive);
  esp_now_register_send_cb(onDataSent);

  // Register broadcast peer
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    loglnE("Failed to add broadcast peer");
  } else {
    loglnI("Broadcast peer registered");
  }

  loglnI("Listening for ESP-NOW packets...");

  // Send first ping immediately on boot
  sendPing();
  lastPing = millis();
}

// ── Loop ─────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  if (now - lastPing >= PING_INTERVAL_MS) {
    lastPing = now;

    char msg[40];
    snprintf(msg, sizeof(msg), "Sending Ping! (uptime: %lus)", now / 1000);
    loglnI(msg);

    sendPing();
  }

  // ESP-NOW receive is interrupt-driven — nothing else needed here
}
