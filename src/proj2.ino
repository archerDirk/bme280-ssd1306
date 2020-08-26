/* This script displays the temperature, humidity and air pressure readings
 * from the BME280 sensor onto an SSD1306 OLED display. 
 */

#include "driver/i2c.h"
#include "Arduino.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "bme280_driver.h"
#include <Wire.h>
#include <stdbool.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() 
{
	Serial.begin(115200);

    Wire.begin(5, 4);
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) 
    {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }    
}
 
void loop() 
{
    char temp[60];
    char humid[60];
    char pres[60];
    bool first_reading = true;
    
    BME280_begin(&Wire);

    while (1)
    {
        set_mode_oversamp();

        /* Ignore the first reading to get stable results. */
        if (first_reading)
        {
            first_reading = false;
        }
        else
        {            
            comp_sensor_data data = { 0 };
            int8_t rslt = BME280_get_sensor_data(&data);

            if (rslt == SUCCESS)
            {          
                /* Convert the temperature member of data.
                 * If data.temp is 2345, then printed value is 23.45
                 */
                sprintf(temp, "* Temp = %.2f *C", data.temp / 100);            
                /* Convert the humidity member of data.
                 * If data.humid is 23450, then printed value is 23.45
                 */
                sprintf(humid, "* Humid = %.2f %%", data.humid / 1000);           
                sprintf(pres, "* Pres = %.2f Pa", data.pres);
            
                print_values(temp, humid, pres);     

                Serial.println("Data retrieved:");      
                Serial.println(temp);
                Serial.println(humid);
                Serial.println(pres);
            }
            else Serial.println("Failed to collect data");
        }
            
        delay(2000);
    }    
}

/* Print the buffers onto the display. */
void print_values(char temp[], char humid[], char pres[])
{   
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, SCREEN_HEIGHT/4);

    display.println(temp);
    display.println(humid);
    display.println(pres);

    delay(2000);
    display.display();
    
    display.clearDisplay();
}
