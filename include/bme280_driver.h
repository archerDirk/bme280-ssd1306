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

#include "driver/i2c.h"
#include <Wire.h>
#include <float.h>


#define SUCCESS 0
#define FAIL -1


#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

/********************************************************/

#ifndef TRUE
#define TRUE                              UINT8_C(1)
#endif
#ifndef FALSE
#define FALSE                             UINT8_C(0)
#endif

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM              UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC               UINT8_C(0x77)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID                    UINT8_C(0x60)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR               UINT8_C(0xD0)
#define BME280_RESET_ADDR                 UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR   UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR              UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR              UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR             UINT8_C(0xF4)
#define BME280_CONFIG_ADDR                UINT8_C(0xF5)
#define BME280_DATA_ADDR                  UINT8_C(0xF7)
#define BME280_REGISTER_STATUS            UINT8_C(0XF3)

/**\name API success code */
#define BME280_OK                         INT8_C(0)

/**\name API error codes */
#define BME280_E_NULL_PTR                 INT8_C(-1)
#define BME280_E_DEV_NOT_FOUND            INT8_C(-2)
#define BME280_E_INVALID_LEN              INT8_C(-3)
#define BME280_E_COMM_FAIL                INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL          INT8_C(-5)
#define BME280_E_NVM_COPY_FAILED          INT8_C(-6)

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO        INT8_C(1)

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN  UINT8_C(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN    UINT8_C(7)
#define BME280_P_T_H_DATA_LEN             UINT8_C(8)

/**\name Sensor power modes */
#define BME280_SLEEP_MODE                 UINT8_C(0x00)
#define BME280_FORCED_MODE                UINT8_C(0x01)
#define BME280_NORMAL_MODE                UINT8_C(0x03)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)            (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)       ((reg_data & (bitname##_MSK)) >> \
                                                  (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

/**\name Macros for bit masking */
#define BME280_SENSOR_MODE_MSK      UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS      UINT8_C(0x00)

#define BME280_CTRL_HUM_MSK         UINT8_C(0x07)
#define BME280_CTRL_HUM_POS         UINT8_C(0x00)

#define BME280_CTRL_PRESS_MSK       UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS       UINT8_C(0x02)

#define BME280_CTRL_TEMP_MSK        UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS        UINT8_C(0x05)

#define BME280_FILTER_MSK           UINT8_C(0x1C)
#define BME280_FILTER_POS           UINT8_C(0x02)

#define BME280_STANDBY_MSK          UINT8_C(0xE0)
#define BME280_STANDBY_POS          UINT8_C(0x05)

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.
 */
#define BME280_PRESS                UINT8_C(1)
#define BME280_TEMP                 UINT8_C(1 << 1)
#define BME280_HUM                  UINT8_C(1 << 2)
#define BME280_ALL                  UINT8_C(0x07)

/**\name Settings selection macros */
#define BME280_OSR_PRESS_SEL        UINT8_C(1)
#define BME280_OSR_TEMP_SEL         UINT8_C(1 << 1)
#define BME280_OSR_HUM_SEL          UINT8_C(1 << 2)
#define BME280_FILTER_SEL           UINT8_C(1 << 3)
#define BME280_STANDBY_SEL          UINT8_C(1 << 4)
#define BME280_ALL_SETTINGS_SEL     UINT8_C(0x1F)

/**\name Oversampling macros */
#define BME280_NO_OVERSAMPLING      UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X      UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X      UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X      UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X      UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X     UINT8_C(0x05)

/**\name Measurement delay calculation macros  */
#define BME280_MEAS_OFFSET          UINT16_C(1250)
#define BME280_MEAS_DUR             UINT16_C(2300)
#define BME280_PRES_HUM_MEAS_OFFSET UINT16_C(575)
#define BME280_MEAS_SCALING_FACTOR  UINT16_C(1000)

/**\name Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS  (0x00)
#define BME280_STANDBY_TIME_62_5_MS (0x01)
#define BME280_STANDBY_TIME_125_MS  (0x02)
#define BME280_STANDBY_TIME_250_MS  (0x03)
#define BME280_STANDBY_TIME_500_MS  (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS   (0x06)
#define BME280_STANDBY_TIME_20_MS   (0x07)

/**\name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF     (0x00)
#define BME280_FILTER_COEFF_2       (0x01)
#define BME280_FILTER_COEFF_4       (0x02)
#define BME280_FILTER_COEFF_8       (0x03)
#define BME280_FILTER_COEFF_16      (0x04)

#define BME280_STATUS_REG_ADDR      (0xF3)
#define BME280_SOFT_RESET_COMMAND   (0xB6)
#define BME280_STATUS_IM_UPDATE     (0x01)

/**\name Calibration registers */
#define BME280_REGISTER_DIG_T1      (0x88)
#define BME280_REGISTER_DIG_T2      (0x8A)
#define BME280_REGISTER_DIG_T3      (0x8C)

#define BME280_REGISTER_DIG_P1      (0x8E)
#define BME280_REGISTER_DIG_P2      (0x90)
#define BME280_REGISTER_DIG_P3      (0x92)
#define BME280_REGISTER_DIG_P4      (0x94)
#define BME280_REGISTER_DIG_P5      (0x96)
#define BME280_REGISTER_DIG_P6      (0x98)
#define BME280_REGISTER_DIG_P7      (0x9A)
#define BME280_REGISTER_DIG_P8      (0x9C)
#define BME280_REGISTER_DIG_P9      (0x9E)

#define BME280_REGISTER_DIG_H1      (0xA1)
#define BME280_REGISTER_DIG_H2      (0xE1)
#define BME280_REGISTER_DIG_H3      (0xE3)
#define BME280_REGISTER_DIG_H4      (0xE4)
#define BME280_REGISTER_DIG_H5      (0xE5)
#define BME280_REGISTER_DIG_H6      (0xE7)

#define BME280_REGISTER_TEMPDATA            (0xFA)
#define BME280_REGISTER_PRESSUREDATA        (0xF7)
#define BME280_REGISTER_HUMIDDATA           (0xFD)

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */


/* Data directly retrieved from the sensor without compensation. */
typedef struct raw_sensor_data 
{
	uint32_t humid;
	uint32_t pres;
	int32_t temp;
} raw_sensor_data;

/* Data from the sensor with compensation. */
typedef struct comp_sensor_data 
{
    float humid;
    float pres;
    float temp;
} comp_sensor_data;

/* Trim Variables */
typedef struct bme280_calibration_data_ 
{
uint16_t dig_t1;
int16_t dig_t2;
int16_t dig_t3;
uint16_t dig_p1;
int16_t dig_p2;
int16_t dig_p3;
int16_t dig_p4;
int16_t dig_p5;
int16_t dig_p6;
int16_t dig_p7;
int16_t dig_p8;
int16_t dig_p9;
uint8_t dig_h1;
int16_t dig_h2;
uint8_t dig_h3;
int16_t dig_h4;
int16_t dig_h5;
int8_t dig_h6;
int32_t t_fine;
} bme280_calibration_data;

typedef struct bme280_settings_sensor
{
    uint8_t osr_p;
    uint8_t osr_t;
    uint8_t osr_h;
    uint8_t filter;
    uint8_t standby_time;
} bme280_settings_sensor;


/* Write data to a register via I2C connection. */
int8_t user_i2c_write(uint8_t dev_addr, 
                      uint8_t reg_addr, 
                      uint8_t *reg_data, 
                      uint8_t cnt);

/* Read out data from a register via I2C connection. */
int8_t user_i2c_read(uint8_t dev_addr, 
                     uint8_t reg_addr, 
                     uint8_t *reg_data, 
                     uint8_t cnt);

/* Set up a I2C connection without the Wire library. */
void initialize_i2c();

/* End the I2C connection without the Wire library. */
void delete_i2c(); 

/* Soft reset BME280. This is done by writing the value 0xB6 [7:0] to 
 * register 0xE0. Register 0xE0 always contains the value 0x00. No reset takes 
 * place for other values than 0xB6.
 */
int8_t BME280_soft_reset();

/* Fills the pressure oversampling settings provided by the user in the data
 * buffer so as to write in the sensor. 
 */
void fill_osr_press_settings(uint8_t *reg_data, 
                             bme280_settings_sensor *settings);

/* Fills the temperature oversampling settings provided by the user in the data 
 * buffer so as to write in the sensor. 
 */
void fill_osr_temp_settings(uint8_t *reg_data, 
                            bme280_settings_sensor *settings);

/* Set the oversampling settings for pressure, temperature and humidity in 
 * the sensor. 
 */
int8_t set_osr_settings(struct bme280_settings_sensor *settings);

/* Set the oversampling settings for humidity readings. */
int8_t set_osr_humidity_settings(bme280_settings_sensor *settings);

/* Set the oversampling settings for pressure and temperature readings. */
int8_t set_osr_press_temp_settings(bme280_settings_sensor *settings);

/* Sets the filter and/or standby duration settings into the sensor. */
int8_t set_filter_standby_settings(bme280_settings_sensor *settings);

/* Fills the filter settings into the sensor. */
void fill_filter_settings(uint8_t *reg_data, bme280_settings_sensor *settings);

/* Fills the standby duration settings into the sensor. */
void fill_standby_settings(uint8_t *reg_data, bme280_settings_sensor *settings);

/* Initialize the BME280 sensor. */
void BME280_init();

/* Get the chip identification number. This is done by reading out register 
 * 0xD0 [7:0]. 
 */
void BME280_getID();

/* Get the temperature and pressure calibration data of the BME280. */
void set_calib_data_temppres(uint8_t *reg_data, bme280_calibration_data *calib_data);

/* Set the humidity calibration data of the BME280. */
void set_calib_data_hum(uint8_t *reg_data, bme280_calibration_data *calib_data);

/* Returns the humidity in percentage * 1000. 23.45% gives the output 
 * value 23450. 
 */
uint32_t compensate_humidity(raw_sensor_data *uncomp_data, 
                             bme280_calibration_data *calib_data);

/* Returns the temperature in Pa. */
uint32_t compensate_pressure(raw_sensor_data *uncomp_data, 
                             bme280_calibration_data *calib_data);

/* Returns the temperature in DegC * 100. 23.45 DegC gives the output 
 * value 2345. 
 */
int32_t compensate_temperature(raw_sensor_data *uncomp_data, 
                               bme280_calibration_data *calib_data);

/* Add compensation to the sensor readings. */
int8_t BME280_compensate_data(raw_sensor_data *uncomp_data, 
                              comp_sensor_data *comp_data);

/* Parse data from the sensor. */
void BME280_parse_sensor_data(const uint8_t *reg_data, 
                              raw_sensor_data *parsed_data);

/* Get the sensor readings. */
int8_t BME280_get_sensor_data(comp_sensor_data *comp_data);

/* Write a byte to a register. */
int8_t write8(uint8_t reg_address, uint8_t *value);

/* Read a byte from a register. */
void read8(uint8_t reg_address, uint8_t *reg_data);

/* Read cnt amount of bytes from a register. */
void read_many(uint8_t reg_address, uint8_t *reg_data, int cnt);

/* Start the Sensor with the Wire library. */
int8_t BME280_begin(TwoWire *theWire);

/* Initialize the sensor with the Wire library. */
int8_t BME280_init_wire();  

/* Check if the sensor is still reading the calibration data. */
int8_t BME280_reading_calibration();

/* Set the mode of the sensor and the oversampling. */
void set_mode_oversamp();
