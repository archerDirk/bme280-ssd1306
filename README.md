# bme280-ssd1306
Weather data (temperature, air pressure, humidity) from BME280 sensor displayed on OLED SSD1306 display.

## proj2.ino
This script displays the temperature, humidity and air pressure readings
from the BME280 sensor onto an SSD1306 OLED display. 

## bme280_driver.h
## bme280_driver.cpp
This script contains the driver for the BME280 sensor. The driver offers 
air pressure, temperature and humidity readings from the sensor. The driver 
files are configured for an ESP32 device.
 
Data is passed on by the i2c protocol. The i2c connection is set up by
either using the i2c.h driver or the Wire library. To use the Wire library,
set the WIRE_ENABLED macro to 1, else the i2c.h driver is used.    

## Sources
This project is based on the Bosch BME280 sensor and the corresponding datasheet:
* https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280-DS002.pdf

The project contains code based on / retrieved from: 
* https://github.com/DarthHTTP/SuperGreenOS
* https://github.com/BoschSensortec/BME280_driver
* https://github.com/adafruit/Adafruit_BME280_Library
