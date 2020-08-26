/* This script contains the driver for the BME280 sensor. The driver offers 
 * air pressure, temperature and humidity readings from the sensor. The driver 
 * files are configured for an ESP32 device.
 * 
 * Data is passed on by the i2c protocol. The i2c connection is set up by
 * either using the i2c.h driver or the Wire library. To use the Wire library,
 * set the WIRE_ENABLED macro to 1, else the i2c.h driver is used.
 * 
 * This project is based on the Bosch BME280 sensor and the corresponding datasheet:
 * - https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280-DS002.pdf
 *
 * The project contains code based on / retrieved from: 
 * - https://github.com/DarthHTTP/SuperGreenOS
 * - https://github.com/BoschSensortec/BME280_driver
 * - https://github.com/adafruit/Adafruit_BME280_Library    
 */

#include "Arduino.h"
#include <stdio.h>
#include <stdlib.h>
#include "bme280_driver.h"

#include <Wire.h>
#include <float.h>

/* Set the Macro to use the Wire library. */
#define WIRE_ENABLED 1

static gpio_num_t i2c_gpio_sda = GPIO_NUM_5;
static gpio_num_t i2c_gpio_scl = GPIO_NUM_4;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

uint8_t dev_id;
uint8_t chip_id_expect;
uint8_t result = -1;
TwoWire *_wire;
int8_t _cs = -1;   
int32_t t_fine_adjust = 0;
bme280_calibration_data calib_data;

/* Write data to a register via I2C connection. */
int8_t user_i2c_write(uint8_t dev_addr, 
                      uint8_t reg_addr, 
                      uint8_t *reg_data, 
                      uint8_t cnt) 
{
    /* Use the Wire library to communicate, else use the i2c driver. */
    if (WIRE_ENABLED)
    {
        if (cnt == 1)
            return write8(reg_addr, reg_data);
        return (int8_t)SUCCESS;	        
    }
    else
    {   
        int32_t iError = 0;

        esp_err_t espRc;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        /* The start signal of the queue with upcoming commands. */
        i2c_master_start(cmd);
        /* Slave address */
        i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
        /* Register address */
        i2c_master_write_byte(cmd, reg_addr, true);
        /* Data byte */
        i2c_master_write(cmd, reg_data, cnt, true);
        /* The stop signal of the queue. */
        i2c_master_stop(cmd);

        /* Send the queue of commands. */
        espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);
        
        if (espRc == ESP_OK) {
            iError = SUCCESS;
        } else {
            iError = FAIL;
        }
        i2c_cmd_link_delete(cmd);

        return (int8_t)iError;
    }
}

/* Read out data from a register via I2C connection. */
int8_t user_i2c_read(uint8_t dev_addr,
                     uint8_t reg_addr, 
                     uint8_t *reg_data, 
                     uint8_t cnt) 
{
    /* Use the Wire library to communicate, else use the i2c driver. */
	if (WIRE_ENABLED)
    {
        if (cnt == 1)
            read8(reg_addr, reg_data);
        else read_many(reg_addr, reg_data, cnt);

        return (int8_t)SUCCESS;
    }
    else
    {   
        int32_t iError = 0;
        esp_err_t espRc;

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        /* The start signal of the queue with upcoming commands. */
        i2c_master_start(cmd);
        /* Slave address. To read the register address has to be sent in 
         * WRITE mode first. 
         */
        i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
        /* Register address */
        i2c_master_write_byte(cmd, reg_addr, true);

        /* Repeated start condition to start READ mode. */
        i2c_master_start(cmd);
        /* Slave address. Now able to be read. */
        i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
        if (cnt > 1) {
            i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);
        

        if (espRc == ESP_OK) {
            iError = SUCCESS;
        } else {
            iError = FAIL;
        }

        i2c_cmd_link_delete(cmd);

        return (int8_t)iError;
    }    
}

/* Set up a I2C connection without the Wire library. */
void initialize_i2c() 
{	
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = i2c_gpio_sda;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = i2c_gpio_scl;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = i2c_frequency;
	i2c_param_config(i2c_port, &conf);
	i2c_driver_install(i2c_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, 
                       I2C_MASTER_TX_BUF_DISABLE, 0);
} 

/* End the I2C connection without the Wire library. */
void delete_i2c() 
{
	i2c_driver_delete(i2c_port);
}

/* Soft reset BME280. This is done by writing the value 0xB6 [7:0] to register 
 * 0xE0. Register 0xE0 always contains the value 0x00. No reset takes place for 
 * other values than 0xB6.
 */
int8_t BME280_soft_reset()
{
    uint8_t reset_value = (uint8_t)BME280_SOFT_RESET_COMMAND;
	int8_t rslt = user_i2c_write(dev_id, BME280_RESET_ADDR, &reset_value, 1);  
    /* Wait for 2 ms to give the bme280 time to reset. */
    delay(2);

    return rslt;
}

/* Fills the pressure oversampling settings provided by the user in the data 
 * buffer so as to write in the sensor. 
 */
void fill_osr_press_settings(uint8_t *reg_data, 
                             bme280_settings_sensor *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

/* Fills the temperature oversampling settings provided by the user in the data
 * buffer so as to write in the sensor. 
 */
void fill_osr_temp_settings(uint8_t *reg_data, bme280_settings_sensor *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

/* Set the oversampling settings for pressure temperature and humidity in 
 * the sensor. 
 */
int8_t set_osr_settings(struct bme280_settings_sensor *settings)
{
    int8_t rslt = BME280_W_INVALID_OSR_MACRO;

    rslt = set_osr_humidity_settings(settings);   
    rslt = set_osr_press_temp_settings(settings);

    return rslt;
}

/* Set the oversampling settings for humidity readings. */
int8_t set_osr_humidity_settings(bme280_settings_sensor *settings)
{
    int8_t rslt;
    uint8_t ctrl_hum;
    uint8_t ctrl_meas;
    uint8_t reg_addr = BME280_CTRL_HUM_ADDR;

    ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;

    /* Write the humidity control value in the register */
    rslt = user_i2c_write(dev_id, reg_addr, &ctrl_hum, 1);

    /* Humidity related changes will be only effective after a
     * write operation to ctrl_meas register
     */
    if (rslt == BME280_OK)
    {
        reg_addr = BME280_CTRL_MEAS_ADDR;
        rslt = user_i2c_read(dev_id, reg_addr, &ctrl_meas, 1);

        if (rslt == BME280_OK)
        {
            rslt = user_i2c_write(dev_id, reg_addr, &ctrl_meas, 1);
        }
    }

    return rslt;
}

/* Set the oversampling settings for pressure and temperature readings. */
int8_t set_osr_press_temp_settings(bme280_settings_sensor *settings)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_CTRL_MEAS_ADDR;
    uint8_t reg_data;

    rslt = user_i2c_read(dev_id, reg_addr, &reg_data, 1);

    if (rslt == BME280_OK)
    {
        fill_osr_press_settings(&reg_data, settings);
        fill_osr_temp_settings(&reg_data, settings);

        /* Write the oversampling settings in the register */
        rslt = user_i2c_write(dev_id, reg_addr, &reg_data, 1);
    }

    return rslt;
}

/* Sets the filter and/or standby duration settings into the sensor. */
int8_t set_filter_standby_settings(bme280_settings_sensor *settings)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_CONFIG_ADDR;
    uint8_t reg_data;

    rslt = user_i2c_read(dev_id, reg_addr, &reg_data, 1);

    if (rslt == BME280_OK)
    {        
        fill_filter_settings(&reg_data, settings);
        fill_standby_settings(&reg_data, settings);  

        /* Write the oversampling settings in the register */
        rslt = user_i2c_write(dev_id, reg_addr, &reg_data, 1);
    }

    return rslt;
}

/* Fills the filter settings into the sensor. */
void fill_filter_settings(uint8_t *reg_data, bme280_settings_sensor *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

/* Fills the standby duration settings into the sensor. */
void fill_standby_settings(uint8_t *reg_data, bme280_settings_sensor *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, 
                                settings->standby_time);
}

/* Initialize the BME280 sensor. */
void BME280_init()
{
    if (WIRE_ENABLED)
    {
        BME280_init_wire();
    }
    else
    {
        dev_id = BME280_I2C_ADDR_PRIM;
        bme280_settings_sensor settings;
        if (BME280_soft_reset() == SUCCESS)
        {        
            delay(10); 
        
            uint8_t reg_data_meas;
            uint8_t reg_data_hum;
            
            if (user_i2c_read(dev_id, BME280_PWR_CTRL_ADDR, &reg_data_meas, 1) == FAIL)
            {
                Serial.println("Fail reg_data_meas");
                return;
            }

            if (user_i2c_read(dev_id, BME280_CTRL_HUM_ADDR, &reg_data_hum, 1) == FAIL)
            {
                Serial.println("Fail reg_data_hum");
                return;
            }   
            else 
            {
                reg_data_hum = BME280_OVERSAMPLING_16X | reg_data_hum;
                user_i2c_write(dev_id, BME280_CTRL_HUM_ADDR, &reg_data_hum, 1);
                delay(10);
                
                /* set mode on bits [0:1] */
                reg_data_meas = BME280_FORCED_MODE | reg_data_meas; 
                /* set osrs_p on bits [2:4] */
                reg_data_meas = (BME280_OVERSAMPLING_16X << 2) | reg_data_meas;
                /* set osrs_t on bits [5:7] */
                reg_data_meas = (BME280_OVERSAMPLING_16X << 5) | reg_data_meas;

                user_i2c_write(dev_id, BME280_PWR_CTRL_ADDR, &reg_data_meas, 1);
                        
                set_filter_standby_settings(&settings);
                delay(50);
                set_osr_settings(&settings);
                delay(50);
            }
        }
    }
}

/* Get the chip identification number. This is done by reading out register 
 * 0xD0 [7:0]. 
 */
void BME280_getID()
{
    chip_id_expect = BME280_CHIP_ID;
    uint8_t chip_id = 0;

    result = user_i2c_read(dev_id, BME280_CHIP_ID_ADDR, &chip_id, 1);
}

/* Set the temperature and pressure calibration data of the BME280. */
void set_calib_data_temppres(uint8_t *reg_data, 
                             bme280_calibration_data *calib_data)
{   
    calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data->dig_h1 = reg_data[25];
}

/* Set the humidity calibration data of the BME280. */
void set_calib_data_hum(uint8_t *reg_data, bme280_calibration_data *calib_data)
{
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
    calib_data->dig_h6 = (int8_t)reg_data[6];
}

/* Returns the humidity in percentage * 1000. 23.45% gives the output 
 * value 23450. 
 */
uint32_t compensate_humidity(raw_sensor_data *uncomp_data, 
                             bme280_calibration_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;

    var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncomp_data->humid * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
    var4 = ((int32_t)calib_data->dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    return humidity;
}

/* Returns the temperature in Pa. */
uint32_t compensate_pressure(raw_sensor_data *uncomp_data, 
                             bme280_calibration_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;

    var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_p6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_p5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data->dig_p4) * 65536);
    var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data->dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_p1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pres;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)calib_data->dig_p9) * ((int32_t)(((pressure / 8) * 
               (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_p8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + 
                   ((var1 + var2 + calib_data->dig_p7) / 16));
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

/* Returns the temperature in DegC * 100. 23.45 DegC gives the output 
 * value 2345. 
 */
int32_t compensate_temperature(raw_sensor_data *uncomp_data, 
                               bme280_calibration_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;

    var1 = (int32_t)((uncomp_data->temp / 8) - ((int32_t)calib_data->dig_t1 * 2));
    var1 = (var1 * ((int32_t)calib_data->dig_t2)) / 2048;
    var2 = (int32_t)((uncomp_data->temp / 16) - ((int32_t)calib_data->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_t3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine * 5 + 128) / 256;

    return temperature;
}

/* Add compensation to the sensor readings. */
int8_t BME280_compensate_data(raw_sensor_data *uncomp_data, 
                              comp_sensor_data *comp_data)
{
    comp_data->temp = compensate_temperature(uncomp_data, &calib_data);
    comp_data->pres = compensate_pressure(uncomp_data, &calib_data);
    comp_data->humid = compensate_humidity(uncomp_data, &calib_data);  

    return SUCCESS;
}

/* Parse data from the sensor. */
void BME280_parse_sensor_data(const uint8_t *reg_data, 
                              raw_sensor_data *parsed_data)
{
    /* Variables to store the sensor data. */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data.
	 * msb + lsb + xlsb = 20 bits. 
	 */

	/* [19:12] bits of parsed data. */
    data_msb = (uint32_t)reg_data[0] << 12;
    /* [11:4] bits of parsed data. */
	data_lsb = (uint32_t)reg_data[1] << 4;
	/* [3:0] bits of parsed data. */
    data_xlsb = (uint32_t)reg_data[2] >> 4;
    parsed_data->pres = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data.
	 * msb + lsb + xlsb = 20 bits. 
	 */

	/* [19:12] bits of parsed data. */
    data_msb = (uint32_t)reg_data[3] << 12;
	/* [11:4] bits of parsed data. */
    data_lsb = (uint32_t)reg_data[4] << 4;
	/* [3:0] bits of parsed data. */
    data_xlsb = (uint32_t)reg_data[5] >> 4;
    parsed_data->temp = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data.
	 * msb + lsb + xlsb = 20 bits. 
	 */

	/* [15:8] bits of parsed data. */
    data_msb = (uint32_t)reg_data[6] << 8;
	/* [7:0] bits of parsed data. */
    data_lsb = (uint32_t)reg_data[7];
    parsed_data->humid = data_msb | data_lsb;
}

/* Get the sensor readings. */
int8_t BME280_get_sensor_data(comp_sensor_data *comp_data)
{
    int8_t rslt;

    /* Array to store the pressure, temperature and humidity data read from
     * the sensor
     */
    uint8_t reg_data[BME280_P_T_H_DATA_LEN] = { 0 };
    raw_sensor_data uncomp_data = { 0 };
    
	/* Read the pressure and temperature data from the sensor */
	rslt = user_i2c_read(dev_id, BME280_DATA_ADDR, reg_data, 
                         BME280_P_T_H_DATA_LEN);
    if (rslt == BME280_OK)
    {
        /* Parse the read data from the sensor */
        BME280_parse_sensor_data(reg_data, &uncomp_data);

        /* Compensate the pressure and/or temperature and/or
         * humidity data from the sensor
         */
        rslt = BME280_compensate_data(&uncomp_data, comp_data);
		
    }
    
    return rslt;
}

/* Write a byte to a register. */
int8_t write8(uint8_t reg_address, uint8_t *value) 
{
    if (_cs == -1) 
    {
        _wire->beginTransmission(BME280_I2C_ADDR_PRIM);
        _wire->write(reg_address);
        _wire->write(*value);
        _wire->endTransmission();
    }
    return SUCCESS;
}

/* Read a byte from a register. */
void read8(uint8_t reg_address, uint8_t *reg_data) 
{
    uint8_t value = 0;

    if (_cs == -1) 
    {
        _wire->beginTransmission(BME280_I2C_ADDR_PRIM);
        _wire->write(reg_address);
        _wire->endTransmission();
        _wire->requestFrom(BME280_I2C_ADDR_PRIM, (unsigned char)1);
        value = _wire->read();
    }

    *reg_data = value;
}

/* Read cnt amount of bytes from a register. */
void read_many(uint8_t reg_address, uint8_t *reg_data, int cnt)
{
    if (_cs == -1) 
    {
        _wire->beginTransmission(BME280_I2C_ADDR_PRIM);
        _wire->write(reg_address);
        _wire->endTransmission();
        _wire->requestFrom(BME280_I2C_ADDR_PRIM, (unsigned char)cnt);

        for (int i = 0; i < cnt - 1; i++)
            reg_data[i]= _wire->read();        
    }    
}

/* Start the Sensor with the Wire library. */
int8_t BME280_begin(TwoWire *theWire) 
{
    int8_t status = -1;
    _wire = theWire;
    status = BME280_init_wire();
  
    return status;
}

/* Initialize the sensor with the Wire library. */
int8_t BME280_init_wire() 
{
    bme280_settings_sensor settings;
    uint8_t reg_data_meas;
    uint8_t reg_data_hum;

    /* init I2C */
    if (_cs == -1) 
    {
        _wire->begin();
    }
    
    BME280_getID();

    if (chip_id_expect != BME280_CHIP_ID)
    {
        Serial.println("Wrong chip id");
        return FAIL;
    }

    BME280_soft_reset();

    /* wait for chip to wake up. */
    delay(10);

    /* if chip is still reading calibration, delay */
    while (BME280_reading_calibration())
        delay(10);

    if (user_i2c_read(dev_id, BME280_PWR_CTRL_ADDR, &reg_data_meas, 1) == FAIL)
    {
        Serial.println("Fail reg_data_meas");
        return FAIL;
    }

    if (user_i2c_read(dev_id, BME280_CTRL_HUM_ADDR, &reg_data_hum, 1) == FAIL)
    {
        Serial.println("Fail reg_data_hum");
        return FAIL;
    }   
    else 
    {   
        reg_data_hum = BME280_OVERSAMPLING_1X | reg_data_hum;
        user_i2c_write(dev_id, BME280_CTRL_HUM_ADDR, &reg_data_hum, 1);
        delay(10);
                
        /* set mode on bits [0:1] */
        reg_data_meas = BME280_FORCED_MODE | reg_data_meas; 
        /* set osrs_p on bits [2:4] */
        reg_data_meas = (BME280_OVERSAMPLING_16X << 2) | reg_data_meas;
        /* set osrs_t on bits [5:7] */
        reg_data_meas = (BME280_OVERSAMPLING_16X << 5) | reg_data_meas;

        user_i2c_write(dev_id, BME280_PWR_CTRL_ADDR, &reg_data_meas, 1);        
    }

    set_filter_standby_settings(&settings);
    delay(50);
    set_osr_settings(&settings);
    delay(100);  

    uint8_t reg_dataAr[BME280_TEMP_PRESS_CALIB_DATA_LEN] = { 0 };
    uint8_t reg_data_humAr[BME280_HUMIDITY_CALIB_DATA_LEN] = { 0 };    

    if (user_i2c_read(dev_id, BME280_TEMP_PRESS_CALIB_DATA_ADDR, reg_dataAr, 
                      BME280_TEMP_PRESS_CALIB_DATA_LEN) == FAIL)
        return FAIL;

    if (user_i2c_read(dev_id, BME280_HUMIDITY_CALIB_DATA_ADDR, reg_data_humAr, 
                      BME280_HUMIDITY_CALIB_DATA_LEN) == FAIL)
        return FAIL;

    set_calib_data_temppres(reg_dataAr, &calib_data);
    set_calib_data_hum(reg_data_humAr, &calib_data);
    delay(100);   

    return SUCCESS;
} 

/* Check if the sensor is still reading the calibration data. */
int8_t BME280_reading_calibration() 
{
    uint8_t rStatus; 
    read8(BME280_REGISTER_STATUS, &rStatus);

    return (rStatus & (1 << 0)) != 0;
}

/* Set the mode of the sensor and the oversampling. */
void set_mode_oversamp()
{
    uint8_t reg_data_meas;
    uint8_t reg_data_hum = 0;
    reg_data_hum = BME280_OVERSAMPLING_1X | reg_data_hum;
    user_i2c_write(dev_id, BME280_CTRL_HUM_ADDR, &reg_data_hum, 1);
    delay(10);
                
    /* set mode on bits [0:1] */
    reg_data_meas = BME280_FORCED_MODE | reg_data_meas; 
    /* set osrs_p on bits [2:4] */
    reg_data_meas = (BME280_OVERSAMPLING_16X << 2) | reg_data_meas;
    /* set osrs_t on bits [5:7] */
    reg_data_meas = (BME280_OVERSAMPLING_16X << 5) | reg_data_meas;

    user_i2c_write(dev_id, BME280_PWR_CTRL_ADDR, &reg_data_meas, 1);
}
