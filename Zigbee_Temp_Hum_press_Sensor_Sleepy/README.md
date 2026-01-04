
## ZigBee - Battery Sensor
### Arduino project setup
Clone repository, and open the ird file.
In the Arduino UI set the following 
* Before Compile/Verify, select the correct board: `Tools -> Board` - **XIAO ESP32c6**
* Select the End device Zigbee mode: `Tools -> Zigbee mode: Zigbee ED (end device)`
* Select Partition Scheme for Zigbee: `Tools -> Partition Scheme: Zigbee 4MB with spiffs`
* Select the COM port: `Tools -> Port: xxx` where the `xxx` is the detected COM port.
* Optional: Set debug level to verbose to see all logs from Zigbee stack: `Tools -> Core Debug Level: Verbose`.

Install the following libraries:
- **Adafruit BME280 Library** and make sure that you install it it might be in the list but not all sub libraries might be installed

If you have changed any of the pins (Battery and the I2C bus). Also if you are using another ESP32c6 dev kit than the Seeed Studio then check  pins accordantly.

Open, compile and upload the code to the ESP32c6.. 

### Connecting the first time
The sensor will be in paring mode when it starts up. this will be after the 10 sec wait if you have reset the device (see bellow) Go and search for it in your Home automation console and pair it. 

### Programming after first run
When the device is paired, it can be hard to re program the device, a 10 seconds delay has been added to the code when a reset is pressed. Meaning if you need to program it wait til the compile is done before upload them press the reset button, the User light (yellow) will come on for the 10 minutes wait.


### Reset to factory
If the device is already registered in a home automation -- Remove ZigBee entry..


![alt text](images/ESP32c6.png)
To reset connection do the following on the ESP32:
<ol>
  <li>Reset (button) & after Boot button, the yellow led (User led) will start blinking fast -- IMPORTANT! Du not use a metal object to push the buttons, it can damage the device if you touches any other terminal on the board. Use a plastic or a nail if your finger is to big to get to the tiny button.
 </li>
  <li>Wait until the yellow light (User led) Stops blinking</li>
  <li>Wait 5 seconds then reset the device and start the search as above</li>
  <li>Start/Restart the ZigBee Device search in Home Assistant</li>
</ol>
