# ESP32BattSensor
This is to build a simple weater station to be used for home automation. The projevt is build for Home Assistans but the ZigBee code can be used genericly.
This is how I have set up my Weather station data

![alt text](<images/Home Assistant - weather station.png>)

There is 2 software versions:
- a ESPHome WiFi based (uses a lot of power)
- a ZigBee version that uses sleep.. Much better with battery life.


## Hardware
This is based on a ESP32C6 from Seeed Studio and a BME280 sensor fro Temperture, Humidity and Barameoric pressure.
