// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Based on a number of exampeles that comes with the Espressif tool kit for Aurdrine.
// 
// For deep down documentation see Github: https://github.com/TheTechDane/ESP32BattSensor
//
// It is based on a Seeed Studio ESP32c6 - and BME280 sensoe and a 18650 battery.
//
//  Make sure you check the Battery pin, and the SLC pins before deployiing the code.
// 

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Zigbee.h"

#define SENSOR_NAME "Mini weather station"
#define SENSOR_MANUFACTURER "The Tech Dane"

#define BATT_EMPTY 2.90
#define BATT_FULL 4.02
#define BATT_PIN A0
#define SDA_PIN D4
#define SCL_PIN D5
#define BME280_ADDR 0x76
#define button BOOT_PIN            //The factory reste pin.
#define USER_LED LED_BUILTIN

Adafruit_BME280 bme; // I2C

/* Zigbee temperature + humidity sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define PRESSURE_SENSOR_ENDPOINT_NUMBER 11

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  55         /* Sleep for 55s will + 5s delay for establishing connection => data reported every 1 minute */

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);
ZigbeePressureSensor zbPressureSensor = ZigbeePressureSensor(PRESSURE_SENSOR_ENDPOINT_NUMBER);

/************************ Temp sensor *****************************/
void meausureAndSleep() {
  float temperature = bme.readTemperature();    // Measure temperature sensor value
  float humidity = bme.readHumidity();          // Messure humidity value
  float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
  int batt = getCurrentBatteryPercent();

  // Update temperature and humidity values in Temperature sensor EP
  zbTempSensor.setTemperature(temperature);
  zbTempSensor.setHumidity(humidity);
  zbTempSensor.report();
  Serial.printf("Reported temperature: %.2f°C, Humidity: %.2f%%\r\n", temperature, humidity);

  zbTempSensor.setBatteryPercentage(batt);  
  zbTempSensor.reportBatteryPercentage();
  Serial.printf("Reported battery: %i%%\r\n", batt);

  Serial.printf("Updating pressure sensor value to %f.0 hPa\r\n", pressure);
  zbPressureSensor.setPressure(pressure);

  // Add small delay to allow the data to be sent before going to sleep
  delay(100);

  // Put device to deep sleep
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  //delay(10000);    //Delay when debugging
}

void blinkLED(int iBlinkDelay = 300) {
    for (int i=0; i<3; i++) {
    digitalWrite(USER_LED, LOW);  // turn the LED on 
    delay(iBlinkDelay);                      
    Serial.println("Blink..");  
    digitalWrite(USER_LED, HIGH);   // turn the LED off 
    delay(iBlinkDelay);   
  }
}

/********************* SETUP **************************/
void setup() {
  Serial.begin(115200);

  // Init button switch
  pinMode(button, INPUT_PULLUP);
  pinMode(USER_LED, OUTPUT);
  pinMode(BATT_PIN, INPUT);         // ADC
  //blinkLED();  //Need logic around sleep.

  //IC2 - Initiate
  delay(1000);
  Wire.begin();
  Wire.beginTransmission(BME280_ADDR);
  int error = Wire.endTransmission();
  if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", BME280_ADDR);
  }

  bool status = bme.begin(BME280_ADDR);
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    //while (1);
  } else {
    Serial.println(F("BME280 Sensor Initialized:"));
  }

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  //Initial TempSensor config
  zbTempSensor.setManufacturerAndModel(SENSOR_MANUFACTURER, SENSOR_NAME);   // Optional: set Zigbee device name and model
  zbTempSensor.setMinMaxValue(10, 50);                                      // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
  zbTempSensor.setTolerance(1);                                             // Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100);                // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) anytime
  zbTempSensor.addHumiditySensor(0, 100, 1);                                // Add humidity cluster to the temperature sensor device with min, max and tolerance values

  //
  // Preassure sensor
  //
  //zbPressureSensor.setManufacturerAndModel("The Tech Dane", "PressureSensor");
  // Set minimum and maximum pressure measurement value in hPa
  zbPressureSensor.setMinMaxValue(0, 10000);
  // Optional: Set tolerance for pressure measurement in hPa
  zbPressureSensor.setTolerance(1);

  // Add both endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);
  Zigbee.addEndpoint(&zbPressureSensor);

  // Create a custom Zigbee configuration for End Device with keep alive 10s to avoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;

  // For battery powered devices, it can be better to set timeout for Zigbee Begin to lower value to save battery
  // If the timeout has been reached, the network channel mask will be reset and the device will try to connect again after reset (scanning all channels)
  Zigbee.setTimeout(10000);  // Set timeout for Zigbee Begin to 10s (default is 30s)

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();  // If Zigbee failed to start, reboot the device and try again
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.println("Successfully connected to Zigbee network");

  // Delay approx 1s (may be adjusted) to allow establishing proper connection with coordinator, needed for sleepy devices
  delay(1000);
}

/************************* LOOP ****************************/
void loop() {
  // Checking button for factory reset
  Serial.println("Button : " + (digitalRead(button) == LOW));
  if (digitalRead(button) == LOW) {  // Push button pressed
    // Key debounce handling
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      blinkLED(100);
      //delay(50);
      if ((millis() - startTime) > 5000) {
        digitalWrite(USER_LED, LOW);  // turn the LED
        // If key pressed for more than 10secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        // Optional set reset in factoryReset to false, to not restart device after erasing nvram, but set it to endless sleep manually instead
        Zigbee.factoryReset(false);
        Serial.println("Going to endless sleep, press RESET button or power off/on the device to wake up");
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_deep_sleep_start();
      }
    }
  }

  // Call the function to measure temperature and put the device to sleep
  meausureAndSleep();
}

float getBatteryVoltage() {
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(BATT_PIN); // ADC with correction   
  }
  float Vbattf = 2* Vbatt / 16 / 1000.0;     // attenuation ratio 1/2, mV --> V
  return Vbattf;
}

int getCurrentBatteryPercent() {
  float Vbattf = getBatteryVoltage();
  Serial.print("Battery Voltage: ");
  Serial.print(Vbattf, 3);

  // Calculate the voltage range
  float voltageRange = BATT_FULL - BATT_EMPTY;
  // Calculate the battery percentage
  int battPercent = ((Vbattf - BATT_EMPTY) / voltageRange) * 100.0;

  Serial.print(" Battery Percentage: ");
  Serial.println(battPercent);

  return battPercent;
}
