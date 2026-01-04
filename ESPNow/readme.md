## ESPNow based Sensor
In the hunt to the best battery performance have I also build a ESPNow version, everyone claims that this is even more power efficient than Zigbee

You are required to have a ESP32 as a Receiver/Gateway in theory be used for more than one Sensor ans also to send commands but the initial version ony one Gateway and one Sensor is used.

The Gateway is a simple ESP32C3 and the Sensor is Identical to the the one in this project for comparing the performance.

HomeAssistant file (ESPHome) cam be found [Here](now-gateway-test.yaml)

The Sensor Code can be found [here](SPNow-Sensor)