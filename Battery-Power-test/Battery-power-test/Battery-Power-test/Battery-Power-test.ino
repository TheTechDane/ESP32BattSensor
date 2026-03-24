//Test Schetch to find the lowest battery curent usage on sleew and on the Battery Monitoring
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define LOGLEVEL LOGLEVEL_DEBUGING
#include <logger.h>

#define BATT_EMPTY 2.90
#define BATT_FULL 4.02
#define BATT_PIN A1
#define BATT_MOSFET_PIN D8
#define SDA_PIN D4
#define SCL_PIN D5
#define BME280_ADDR 0x76
#define button BOOT_PIN            //The factory reste pin.
#define USER_LED LED_BUILTIN
#define nrBattSamples 5            //Battery meassurement - Was 16 to start with
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
//#define TIME_TO_SLEEP  115         /* Sleep for 115s will + 5s delay for establishing connection => data reported every 2 minutes */
#define TIME_TO_SLEEP  30         /* Sleep for 20s for testing */
#define RESET_DELAY 10            /* If reset was pressed wait 10 sec, if it was for programming */
long t = 0;
Adafruit_BME280 bme; // I2C
// A variable to hold the reset cause flags
bool wasResetRestart = false;
int rebootTime = 0;
bool readyToSleep = false;

typedef struct sensorData {
  char senderID[11];
  int type = 1;    // Type of payload
  float temp;       // Temp
  float humi;       // Humitity
  int pres;         // Preassure in mBar
  int batt;         // Battery Presentage
} sensorData;

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
  
  pinMode(USER_LED , OUTPUT);
  digitalWrite(USER_LED, HIGH); 
  pinMode(BATT_MOSFET_PIN, OUTPUT);
  digitalWrite(BATT_MOSFET_PIN, LOW);


  // Configure the wake up source and set to wake up every xx secunds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  //IC2 - Initiate  -- This originally sat in the DoMessurement function but was mowed as it does not make a differance when DeepSleeping
  Wire.begin();
  Wire.beginTransmission(BME280_ADDR);
  int error = Wire.endTransmission();
  if (error == 0) {
      logD("I2C device found at address:");
      loglnD(BME280_ADDR);
  }
  bool status = bme.begin(BME280_ADDR);
  if (!status) {
    loglnD("Could not find a valid BME280 sensor, check wiring!");
    //while (1);
  } else {
    loglnD("BME280 Sensor Initialized:");
  }
}

/******************************************************************
*
*  Do the Actual Measurement and send it to the gateway.
*
******************************************************************/
void doMeasurement() {
  sensorData mySensorData;                      // The Sensor data sending
  //For loop for testing the reads - Should only be one.
  for (int i=0; i<1;i++) {
      delay(150);    //MAke sure Sensor is ready
      mySensorData.temp = 10; //bme.readTemperature();    // Measure temperature sensor value
      mySensorData.humi = 80; //bme.readHumidity();          // Messure humidity value
      mySensorData.pres = 1003; //round(bme.readPressure() / 100.0F); // Convert Pa to hPa
      loglnD("Batt pin A1");
      mySensorData.batt = getCurrentBatteryPercent(A1);
      loglnD("Batt pin A2");
      mySensorData.batt = getCurrentBatteryPercent(A2);

      char sFormatted[100];
      sprintf(sFormatted,"Meausured Temp:%1f Humitity:%1f Preassure:%d Batt:%d", mySensorData.temp, mySensorData.humi, mySensorData.pres, mySensorData.batt);
      loglnD(sFormatted);
  }
}

/******************************* LOOP ************************/
void loop() {
  //Check if reset was pressed - and time has been passed
  if ((millis() - rebootTime) < (TIME_TO_SLEEP  * 1000)) {
    digitalWrite(BATT_MOSFET_PIN, !digitalRead(BATT_MOSFET_PIN));
    blinkLEDOnce();
    doMeasurement();  //Might as well send meassurement when we wait roe the reboot periode (For programming etc.)
    delay(1000);
  } else {
    // Call the function to measure temperature and put the device to sleep
    blinkLEDOnce();
    doMeasurement();
    delay(100);    //To ensure Send
    gotoSleep();
  }
}


float getBatteryVoltage(int adc_pin) {
  uint32_t Vbatt = 0;
  for(int i = 0; i < nrBattSamples; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(adc_pin); // ADC with correction   
  }
  float Vbattf = 2* Vbatt / nrBattSamples / 1000.0;     // attenuation ratio 1/2, mV --> V
  return Vbattf;
}

int getCurrentBatteryPercent(int adc_pin) {
  float Vbattf = getBatteryVoltage(adc_pin);
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
