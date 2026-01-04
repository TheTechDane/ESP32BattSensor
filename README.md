# ESP32BattSensor
This is to build a simple weather station to be used for home automation. The project is build for Home Assistant but the ZigBee code can be used generic.
This is how I have set up my Weather station data

![alt text](<images/Home Assistant - weather station.png>)

There is 3 software versions:
- a ESPHome WiFi based (uses a lot of power)
- a ZigBee version that uses sleep.. Much better with battery life.
- a ESPnow based system require a ESP as gate way as well (Still in test)

The reason for the 3 projects are to test range and battery usage. The are all based on the same Battery Sensor:

For details see bellow on the Hardware section.

The general result is that .........




## Hardware
This is based on a ESP32C6 from Seeed Studio and a BME280 sensor fro Temperature, Humidity and Barometric pressure.

##Detales on the projects can be found here: [ESPHome (WiFi)](ESPHome) [ZigBee](Zigbee_Temp_Hum_press_Sensor_Sleepy) [ESPNow (ESPHome based)](ESPNow)

