/*
    Name:    ESPNow Sensor (BME280 + Battery + Auto-Channel + USB Debug)
    Date:    2026 - November
    Author:  TheTechDane aka - Kim Rasmussen
    Version: 0.3.0
    Board:   Seeed Studio XIAO ESP32-C6
             (Ensure Tools -> USB CDC On Boot -> Enabled)
*/
#define VER "0.3.0"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_mac.h>
#include "driver/usb_serial_jtag.h"

#define LOGLEVEL LOGLEVEL_DEBUGING
#include <logger.h>

// =========================================================================
// CONFIGURATION
// =========================================================================
uint8_t broadcastAddress[] = {0x64, 0xE8, 0x33, 0x84, 0x63, 0x90};
const char senderID[11] = "ESPNow-001";

#define DEFAULT_CHANNEL   11
#define EXTERNAL_ANTENNA  true

// Hardware Pins (XIAO ESP32-C6)
#define BATT_PIN          A0
#define SDA_PIN           D4
#define SCL_PIN           D5
#define BME280_ADDR       0x76
#define USER_LED          LED_BUILTIN

// Battery Calibration (Single-cell LiPo)
#define BATT_EMPTY        2.75f
#define BATT_FULL         4.05f
#define NR_BATT_SAMPLES   5

// Timing & Sleep Settings
#define uS_TO_S_FACTOR        1000000ULL
#define TIME_TO_SLEEP         119       // ~2 minute total cycle on battery
#define USB_AWAKE_WINDOW_MS   60000UL   // Stay awake 60s when USB is connected
#define BATT_RESET_WINDOW_MS  5000UL    // Brief 5s window on battery cold boot/reset

// =========================================================================
// DATA STRUCTURES & GLOBALS
// =========================================================================
typedef struct sensorData {
  char senderID[11];
  int32_t type = 1;       // Payload type
  float temp;             // Temperature (°C)
  float humi;             // Humidity (%)
  int32_t pres;           // Pressure (hPa / mBar)
  int32_t batt;           // Battery percentage (0-100%)
  float battVoltage;      // Battery voltage (V)
} sensorData;

// Persists in RTC slow memory across Deep Sleep wakes (resets only on power loss)
RTC_DATA_ATTR uint8_t cachedChannel = DEFAULT_CHANNEL;

esp_now_peer_info_t peerInfo;
Adafruit_BME280 bme;
bool bmeAvailable = false;

unsigned long awakeUntil = 0;
bool usbDebugActive = false;
volatile bool readyToSleep = false;
volatile int lastSendStatus = -1; // -1 = waiting, 0 = ACK OK, 1 = NO ACK

// =========================================================================
// HELPER FUNCTIONS
// =========================================================================
bool isUsbConnected() {
  return usb_serial_jtag_is_connected() || (bool)Serial;
}

void blinkLEDOnce(int iBlinkDelay = 100) {
  digitalWrite(USER_LED, LOW);  // LED ON (active LOW on XIAO)
  delay(iBlinkDelay);
  digitalWrite(USER_LED, HIGH); // LED OFF
  delay(iBlinkDelay);
}

void printMacAddress() {
  uint8_t baseMac[6];
  if (esp_read_mac(baseMac, ESP_MAC_WIFI_STA) == ESP_OK) {
    char cFormattedMac[24];
    snprintf(cFormattedMac, sizeof(cFormattedMac), "%02X:%02X:%02X:%02X:%02X:%02X",
             baseMac[0], baseMac[1], baseMac[2],
             baseMac[3], baseMac[4], baseMac[5]);
    logD("Sender STA MAC: ");
    loglnD(cFormattedMac);
  } else {
    loglnE("Error getting MAC address!");
  }
}

float getBatteryVoltage() {
  uint32_t Vbatt = 0;
  for (int i = 0; i < NR_BATT_SAMPLES; i++) {
    Vbatt += analogReadMilliVolts(BATT_PIN);
  }
  return (2.0f * Vbatt) / NR_BATT_SAMPLES / 1000.0f;
}

int getCurrentBatteryPercent(float Vbattf) {
  if (Vbattf <= BATT_EMPTY) return 0;
  if (Vbattf >= BATT_FULL) return 100;
  return round(((Vbattf - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY)) * 100.0f);
}

// =========================================================================
// ESP-NOW CHANNEL & CALLBACKS
// =========================================================================
void setEspNowChannel(uint8_t ch) {
  WiFi.setChannel(ch);
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ch;
  peerInfo.encrypt = false;

  if (esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_mod_peer(&peerInfo);
  } else {
    esp_now_add_peer(&peerInfo);
  }
}

uint8_t autoDetectGatewayChannel() {
  if (usbDebugActive) Serial.println("[SCAN] Sweeping ESP-NOW channels 1-13 for Gateway ACK...");
  uint8_t pingByte = 0x00; // 1-byte probe (ignored by struct size check on gateway)

  for (uint8_t ch = 1; ch <= 13; ch++) {
    setEspNowChannel(ch);
    lastSendStatus = -1;
    esp_now_send(broadcastAddress, &pingByte, sizeof(pingByte));

    unsigned long start = millis();
    while (lastSendStatus == -1 && (millis() - start < 15)) {
      delay(1);
    }

    if (lastSendStatus == 0) { // Gateway hardware returned 802.11 ACK!
      cachedChannel = ch;
      if (usbDebugActive) Serial.printf("[SCAN] Found Gateway on Channel %d!\n", ch);
      return ch;
    }
  }

  // Restore previous cached channel if nothing answered
  setEspNowChannel(cachedChannel);
  if (usbDebugActive) Serial.println("[SCAN] No Gateway found on channels 1-13.");
  return 0;
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  lastSendStatus = (status == ESP_NOW_SEND_SUCCESS) ? 0 : 1;
  readyToSleep = true;
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (usbDebugActive) {
    Serial.printf("[RECV] %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n", len,
                  recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                  recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
  }
}

// =========================================================================
// MEASUREMENT & SLEEP
// =========================================================================
void transmitSensorData(const sensorData &payload) {
  readyToSleep = false;
  lastSendStatus = -1;
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&payload, sizeof(payload));

  if (result != ESP_OK) {
    loglnE("esp_now_send() queueing failed!");
    readyToSleep = true;
    lastSendStatus = 1;
    return;
  }

  unsigned long txStart = millis();
  while (!readyToSleep && (millis() - txStart < 50)) {
    delay(1);
  }
}

void doMeasurement() {
  sensorData mySensorData;
  strncpy(mySensorData.senderID, senderID, sizeof(mySensorData.senderID));
  mySensorData.senderID[sizeof(mySensorData.senderID) - 1] = '\0';

  if (bmeAvailable && bme.takeForcedMeasurement()) {
    mySensorData.temp = bme.readTemperature();
    mySensorData.humi = bme.readHumidity();
    mySensorData.pres = round(bme.readPressure() / 100.0F);
  } else {
    mySensorData.temp = NAN;
    mySensorData.humi = NAN;
    mySensorData.pres = 0;
    loglnE("BME280 read failed or sensor missing!");
  }

  mySensorData.battVoltage = getBatteryVoltage();
  mySensorData.batt = getCurrentBatteryPercent(mySensorData.battVoltage);

  char sFormatted[128];
  snprintf(sFormatted, sizeof(sFormatted),
           "Measured -> Temp: %.1fC | Humi: %.1f%% | Pres: %dhPa | Batt: %d%% (%.2fV)",
           mySensorData.temp, mySensorData.humi, mySensorData.pres,
           mySensorData.batt, mySensorData.battVoltage);
  loglnD(sFormatted);

  // 1. Send on cachedChannel first (0ms scan overhead during normal operation)
  transmitSensorData(mySensorData);

  // 2. If no ACK came back, gateway likely switched channels -> auto-detect & retry once
  if (lastSendStatus != 0) {
    if (usbDebugActive) Serial.printf("[WARN] No ACK on Ch %d -> running channel sweep...\n", cachedChannel);
    if (autoDetectGatewayChannel() > 0) {
      transmitSensorData(mySensorData);
    }
  }
}

void gotoSleep() {
  loglnI("Entering Deep Sleep...");
  if (usbDebugActive) {
    Serial.flush();
  }
  esp_deep_sleep_start();
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, HIGH); // LED OFF

  delay(200); // Brief settle time for USB SOF detection

  esp_reset_reason_t iBootReason = esp_reset_reason();

  if (isUsbConnected()) {
    usbDebugActive = true;
    awakeUntil = millis() + USB_AWAKE_WINDOW_MS;
    delay(1500);
    Serial.println("\n==========================================");
    Serial.printf(" ESPNow Sensor v%s [USB DEBUG MODE]\n", VER);
    Serial.println(" Staying awake for 60s before Deep Sleep");
    Serial.println("==========================================");
  } else if (iBootReason == ESP_RST_POWERON || iBootReason == ESP_RST_EXT) {
    awakeUntil = millis() + BATT_RESET_WINDOW_MS;
  } else {
    awakeUntil = 0; // Normal Deep Sleep wake on battery
  }

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Configure XIAO ESP32-C6 RF Switch before initializing radio
  if (EXTERNAL_ANTENNA) {
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);  // RF switch power ON
    pinMode(14, OUTPUT);
    digitalWrite(14, HIGH); // Select external u.FL antenna
  }

  // Initialize I2C and BME280 in Forced Mode
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bme.begin(BME280_ADDR, &Wire)) {
    loglnE("Could not find a valid BME280 sensor at 0x76!");
    bmeAvailable = false;
  } else {
    bmeAvailable = true;
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_OFF);
  }

  // Initialize radio in STA mode (pure ESP-NOW, no Wi-Fi AP connection)
  WiFi.mode(WIFI_STA);

  if (usbDebugActive) {
    printMacAddress();
    Serial.printf("Target Gateway: %02X:%02X:%02X:%02X:%02X:%02X | Cached Ch: %d\n\n",
                  broadcastAddress[0], broadcastAddress[1], broadcastAddress[2],
                  broadcastAddress[3], broadcastAddress[4], broadcastAddress[5],
                  cachedChannel);
  }

  if (esp_now_init() != ESP_OK) {
    loglnE("ESP-NOW init failed!");
    gotoSleep();
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Lock radio and peer to cachedChannel
  setEspNowChannel(cachedChannel);
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  if (!usbDebugActive && isUsbConnected()) {
    usbDebugActive = true;
    awakeUntil = millis() + USB_AWAKE_WINDOW_MS;
    Serial.println("\n[USB CONNECTED] Extending awake window to 60s...");
  }

  if (millis() < awakeUntil) {
    doMeasurement();

    if (usbDebugActive) {
      unsigned long secLeft = (awakeUntil > millis()) ? (awakeUntil - millis()) / 1000 : 0;
      Serial.printf("Gateway ACK: %s (Ch %d) | Deep Sleep in %lus\n\n",
                    (lastSendStatus == 0) ? "OK" : "FAILED",
                    cachedChannel, secLeft);
    }

    if (lastSendStatus == 0) {
      blinkLEDOnce(80); // 1 flash = OK
    } else {
      for (int i = 0; i < 3; i++) blinkLEDOnce(60); // 3 flashes = No ACK
    }

    delay(3000);
  } else {
    doMeasurement();
    gotoSleep();
  }
}