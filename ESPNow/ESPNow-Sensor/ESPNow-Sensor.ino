/*
    Name: ESPNow Sensor
    Date: 2026 - January
    Authoe: TheTechDane aka - Kim Rasmussen
    Version: */ 
#define VER "0.0.1"
/*
    This is a Simple Sensor to broadcast Temp, Humitity and Preassure
    This is the first version aimed to be as simple as possiable to transfer data to HomeAssistanc over a ESPHome gateway

    For now It will send the data 
    And if anything is recieved just log it on the serial port

    Later this will be the foundation for a battery powered sencor with DeepSleep to save battery

    Helpfull links: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

*/
#include "ESP32_NOW.h"
#include "WiFi.h"
#define LOGLEVEL LOGLEVEL_DEBUGING
#include <logger.h>

//IMPORTANT to set the right MAC address of the reciving gateway..
uint8_t broadcastAddress[] = {0x34, 0x85, 0x18, 0x00, 0x2D, 0xE4};
// Broadcast -- uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char senderID[11] = "ESPNow-tst";
#define ESPNowChannal 1

typedef struct sensorData {
  char senderID[11];
  int type = 1;    // Type of payload
  float temp;       // Temp
  float humi;       // Humitity
  int pres;         // Preassure in mBar
  int batt;         // Battery Presentage
} sensorData;

sensorData mySensorData;              // The Sensor data sending
esp_now_peer_info_t peerInfo;         // For the payload that returns, and the peer registration.
long t = 0;

void readMacAddress(){
  uint8_t baseMac[6];
  char cFormattedMac[21];
  esp_err_t ret = esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  if (ret == ESP_OK) {
    sprintf(cFormattedMac,"%02x:%02x:%02x:%02x:%02x:%02x\n",
      baseMac[0], baseMac[1], baseMac[2],
      baseMac[3], baseMac[4], baseMac[5]);
  } else {
    loglnE("Error getting MAC address!!");
  }
  logD("MAC Address:");
  loglnD(cFormattedMac);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Current log level is :" + String(LOGLEVEL));
  loglnV("Starting");
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.mode(WIFI_STA);   //Use special ESPNow Mode
  
  //Start ESPNow
  if (esp_now_init() != ESP_OK) {
    loglnE("ESPNOW FAILED TO START!");
    readMacAddress();
    return;
  }else{
     loglnI("ESPNOW started!");
  }
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNowChannal;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    loglnE("ESPNOW FAILED TO to add peer");
    return;
  }
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  //esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - t > 2000){
    t = millis();

    strncpy(mySensorData.senderID, senderID, sizeof(senderID));
    mySensorData.type = 1;    // Type of payload
    mySensorData.temp = random(2000,5000)/100.0;//20.00 ~ 50.00
    mySensorData.humi = random(2000,5000)/100.0;
    mySensorData.pres = 1000;
    mySensorData.batt = 67;

    esp_now_send(broadcastAddress, (uint8_t *) &mySensorData, sizeof(mySensorData));
  }
}

/******************************************************************************
  OnDataSend callback
  Status if data was send correctly
*******************************************************************************/
void OnDataSent(uint8_t *mac, esp_now_send_status_t status) {

  char cFormattedMac[21];
  sprintf(cFormattedMac,"%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0], mac[1], mac[2],mac[3],mac[4],mac[5]);
  logD("Sent to MAC Address:");
  loglnD(cFormattedMac);



logI("Send status:");
loglnI(status);

   if(status == ESP_NOW_SEND_SUCCESS) {
     loglnI("Send Successfull!");
   }
   else {
     loglnE("SEND Failed!");
   }
}

/******************************************************************************
  OnRecieve call back

  TODO : Filter the data if it is for "me"
*******************************************************************************/
void OnDataRecv(uint8_t * mac, uint8_t * data, uint8_t len) {

}
