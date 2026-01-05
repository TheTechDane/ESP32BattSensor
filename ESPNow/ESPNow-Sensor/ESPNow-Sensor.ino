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
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#define LOGLEVEL LOGLEVEL_DEBUGING
#include <logger.h>

//IMPORTANT to set the right MAC address of the reciving gateway..
uint8_t broadcastAddress[] = {0x34, 0x85, 0x18, 0x00, 0x2D, 0xE4};
// Broadcast -- uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char senderID[11] = "ESPNow-001";
#define ESPNowChannal 1

#define BATT_EMPTY 2.90
#define BATT_FULL 4.02
#define BATT_PIN A0
#define SDA_PIN D4
#define SCL_PIN D5
#define BME280_ADDR 0x76
#define button BOOT_PIN            //The factory reste pin.
#define USER_LED LED_BUILTIN
#define nrBattSamples 5            //Battery meassurement - Was 16 to start with
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
//#define TIME_TO_SLEEP  115         /* Sleep for 115s will + 5s delay for establishing connection => data reported every 2 minutes */
#define TIME_TO_SLEEP  20         /* Sleep for 20s for testing */
#define RESET_DELAY 10            /* If reset was pressed wait 10 sec, if it was for programming */

typedef struct sensorData {
  char senderID[11];
  int type = 1;    // Type of payload
  float temp;       // Temp
  float humi;       // Humitity
  int pres;         // Preassure in mBar
  int batt;         // Battery Presentage
} sensorData;

esp_now_peer_info_t peerInfo;         // For the payload that returns, and the peer registration.
long t = 0;
Adafruit_BME280 bme; // I2C
// A variable to hold the reset cause flags
bool wasResetRestart = false;
int rebootTime = 0;
bool readyToSleep = false;

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

/******************************************************************
*
*  Do the Actual Measurement 
*
******************************************************************/
void doMeasurement() {
  //IC2 - Initiate
  Wire.begin();
  Wire.beginTransmission(BME280_ADDR);
  int error = Wire.endTransmission();
  if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", BME280_ADDR);
  }

  bool status = bme.begin(BME280_ADDR);
  if (!status) {
    loglnD("Could not find a valid BME280 sensor, check wiring!");
    //while (1);
  } else {
    loglnD("BME280 Sensor Initialized:");
  }

  sensorData mySensorData;                      // The Sensor data sending
  strncpy(mySensorData.senderID, senderID, sizeof(senderID));
  mySensorData.temp = bme.readTemperature();    // Measure temperature sensor value
  mySensorData.humi = bme.readHumidity();          // Messure humidity value
  mySensorData.pres = round(bme.readPressure() / 100.0F); // Convert Pa to hPa
  mySensorData.batt = getCurrentBatteryPercent();

  char sFormatted[100];
  sprintf(sFormatted,"Meausured Temp:%1f Humitity:%1f Preassure:%d Batt:%d", mySensorData.temp, mySensorData.humi, mySensorData.pres, mySensorData.batt);
  loglnD(sFormatted);

  //Send
  esp_now_send(broadcastAddress, (uint8_t *) &mySensorData, sizeof(mySensorData));
}

void gotoSleep() {
  //Put Sensor to sleep
  BME280_Sleep(BME280_ADDR);
  // Put device to deep sleep
  loglnI("Going to sleep now");
  esp_deep_sleep_start();
}

void blinkLEDOnce(int iBlinkDelay = 300) {
    digitalWrite(USER_LED, LOW);  // turn the LED on 
    delay(iBlinkDelay);                      
    loglnD("Blink..");  
    digitalWrite(USER_LED, HIGH);   // turn the LED off 
    delay(iBlinkDelay);   
}

void blinkLED(int iBlinkDelay = 300) {
    for (int i=0; i<3; i++) {
      blinkLEDOnce(iBlinkDelay);
  }
}

/********************* SETUP **************************/
void setup() {
  Serial.begin(115200);
  Serial.println("Current log level is :" + String(LOGLEVEL));
  loglnV("Starting");
 
   //
  // Check Reboot reason
  //
  int iBootReason = esp_reset_reason();               // but I still can use and find the ummerical value
  logD("Reset/Boot Reason was: "); loglnD( iBootReason );
  if ( iBootReason == ESP_RST_POWERON ) {             // Reset due to power-on event.
    logD("Reboot was because of Power-On!!  - Waiting ");
    logD(RESET_DELAY);
    loglnD(" Sec.");
    wasResetRestart = true;
  }
  if ( iBootReason == ESP_RST_DEEPSLEEP) {                  // Reset due to power-on event.
    loglnD("Reboot was because of DeepSleep!!");
  }
  if ( iBootReason == ESP_RST_EXT ) {                  // Reset due to power-on event.
    loglnD("Reboot was because of External reset!!");
  }
  rebootTime = millis();
  
  // Configure the wake up source and set to wake up every xx secunds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
 
 
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

/******************************* LOOP ************************/
void loop() {
  // put your main code here, to run repeatedly:
  if (wasResetRestart) {
    //Check if reset was pressed - and time has been passed
    if ((millis() - rebootTime) < (RESET_DELAY * 1000)) {
      blinkLED(1000);
    } else {
      wasResetRestart = false;
    }
  } else {
    // Call the function to measure temperature and put the device to sleep
    blinkLEDOnce();
    doMeasurement();
    delay(100);    //To ensure Send
    gotoSleep();
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

  readyToSleep = true;

   if(status == ESP_NOW_SEND_SUCCESS) {   //Not sure why I get a 1 and it does send !! ?? !!
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


float getBatteryVoltage() {
  uint32_t Vbatt = 0;
  for(int i = 0; i < nrBattSamples; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(BATT_PIN); // ADC with correction   
  }
  float Vbattf = 2* Vbatt / nrBattSamples / 1000.0;     // attenuation ratio 1/2, mV --> V
  return Vbattf;
}

int getCurrentBatteryPercent() {
  float Vbattf = getBatteryVoltage();
  logD("Battery Voltage: ");
  logD(Vbattf);

  // Calculate the voltage range
  float voltageRange = BATT_FULL - BATT_EMPTY;
  // Calculate the battery percentage
  int battPercent = 0;
  if ((Vbattf - BATT_EMPTY) > 0.0) battPercent = ((Vbattf - BATT_EMPTY) / voltageRange) * 100.0;

  logD(" Battery Percentage: ");
  loglnD(battPercent);

  return battPercent;
}

void BME280_Sleep(int device_address) {
  // FROM: https://github.com/G6EJD/BME280-Sleep-and-Address-change/blob/master/BME280_Sleep.ino
  //
  // BME280 Register 0xF4 (control measurement register) sets the device mode, specifically bits 1,0
  // The bit positions are called 'mode[1:0]'. See datasheet Table 25 and Paragraph 3.3 for more detail.
  // Mode[1:0]  Mode
  //    00      'Sleep'  mode
  //  01 / 10   'Forced' mode, use either '01' or '10'
  //    11      'Normal' mode
  loglnI("BME280 to Sleep mode...");
  Wire.beginTransmission(device_address);
  Wire.write((uint8_t)0xF4);       // Select Control Measurement Register
  Wire.write((uint8_t)0b00000000); // Send '00' for Sleep mode
  Wire.endTransmission();
}
