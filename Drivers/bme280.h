/*!
 * @file Adafruit_BME280.h
 *
 * Designed specifically to work with the Adafruit BME280 Breakout
 * ----> http://www.adafruit.com/products/2650
 *
 * These sensors use I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 * See the LICENSE file for details.
 *
 */

#ifndef __BME280_H__
#define __BME280_H__
#include "main.h"

/*!
 *  @brief  default I2C address
 */
#define BME280_ADDRESS (0x77)           // Primary I2C Address
                                        /*!
                                         *  @brief  alternate I2C address
                                         */
#define BME280_ADDRESS_ALTERNATE (0x76) // Alternate Address

/*!
 *  @brief Register addresses
 */
enum {
  BME280_REGISTER_DIG_T1 = 0x88,
  BME280_REGISTER_DIG_T2 = 0x8A,
  BME280_REGISTER_DIG_T3 = 0x8C,

  BME280_REGISTER_DIG_P1 = 0x8E,
  BME280_REGISTER_DIG_P2 = 0x90,
  BME280_REGISTER_DIG_P3 = 0x92,
  BME280_REGISTER_DIG_P4 = 0x94,
  BME280_REGISTER_DIG_P5 = 0x96,
  BME280_REGISTER_DIG_P6 = 0x98,
  BME280_REGISTER_DIG_P7 = 0x9A,
  BME280_REGISTER_DIG_P8 = 0x9C,
  BME280_REGISTER_DIG_P9 = 0x9E,

  BME280_REGISTER_DIG_H1 = 0xA1,
  BME280_REGISTER_DIG_H2 = 0xE1,
  BME280_REGISTER_DIG_H3 = 0xE3,
  BME280_REGISTER_DIG_H4 = 0xE4,
  BME280_REGISTER_DIG_H5 = 0xE5,
  BME280_REGISTER_DIG_H6 = 0xE7,

  BME280_REGISTER_CHIPID = 0xD0,
  BME280_REGISTER_VERSION = 0xD1,
  BME280_REGISTER_SOFTRESET = 0xE0,

  BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

  BME280_REGISTER_CONTROLHUMID = 0xF2,
  BME280_REGISTER_STATUS = 0XF3,
  BME280_REGISTER_CONTROL = 0xF4,
  BME280_REGISTER_CONFIG = 0xF5,
  BME280_REGISTER_PRESSUREDATA = 0xF7,
  BME280_REGISTER_TEMPDATA = 0xFA,
  BME280_REGISTER_HUMIDDATA = 0xFD
};

/**************************************************************************/
/*!
    @brief  calibration data
*/
/**************************************************************************/
typedef struct {
  uint16_t dig_T1; ///< temperature compensation value
  int16_t dig_T2;  ///< temperature compensation value
  int16_t dig_T3;  ///< temperature compensation value

  uint16_t dig_P1; ///< pressure compensation value
  int16_t dig_P2;  ///< pressure compensation value
  int16_t dig_P3;  ///< pressure compensation value
  int16_t dig_P4;  ///< pressure compensation value
  int16_t dig_P5;  ///< pressure compensation value
  int16_t dig_P6;  ///< pressure compensation value
  int16_t dig_P7;  ///< pressure compensation value
  int16_t dig_P8;  ///< pressure compensation value
  int16_t dig_P9;  ///< pressure compensation value

  uint8_t dig_H1; ///< humidity compensation value
  int16_t dig_H2; ///< humidity compensation value
  uint8_t dig_H3; ///< humidity compensation value
  int16_t dig_H4; ///< humidity compensation value
  int16_t dig_H5; ///< humidity compensation value
  int8_t dig_H6;  ///< humidity compensation value
} bme280_calib_data;
/*=========================================================================*/



/** Adafruit Unified Sensor interface for temperature component of BME280 */

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with BME280 IC
*/
/**************************************************************************/


  /**************************************************************************/
  /*!
      @brief  sampling rates
  */
  /**************************************************************************/
  typedef enum {
    SAMPLING_NONE = 0b000,
    SAMPLING_X1 = 0b001,
    SAMPLING_X2 = 0b010,
    SAMPLING_X4 = 0b011,
    SAMPLING_X8 = 0b100,
    SAMPLING_X16 = 0b101
  }sensor_sampling;

  /**************************************************************************/
  /*!
      @brief  power modes
  */
  /**************************************************************************/
  typedef enum {
    MODE_SLEEP = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
  }sensor_mode;

  /**************************************************************************/
  /*!
      @brief  filter values
  */
  /**************************************************************************/
  typedef enum  {
    FILTER_OFF = 0b000,
    FILTER_X2 = 0b001,
    FILTER_X4 = 0b010,
    FILTER_X8 = 0b011,
    FILTER_X16 = 0b100
  }sensor_filter;

  /**************************************************************************/
  /*!
      @brief  standby duration in ms
  */
  /**************************************************************************/
  typedef enum  {
    STANDBY_MS_0_5 = 0b000,
    STANDBY_MS_10 = 0b110,
    STANDBY_MS_20 = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125 = 0b010,
    STANDBY_MS_250 = 0b011,
    STANDBY_MS_500 = 0b100,
    STANDBY_MS_1000 = 0b101
  }standby_duration;
  /**************************************************************************/
  /*!
      @brief  config register
  */
  /**************************************************************************/
  typedef struct  {
		
		// unused - don't set
    unsigned int spi3w_en : 1; ///< unused - don't set
    

		
    unsigned int none : 1;     ///< unused - don't set
    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    unsigned int filter : 3; ///< filter settings
// inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    unsigned int t_sb : 3; ///< inactive duration (standby time) in normal mode
    

    /// @return combined config register
   
  }config;
  extern config _configReg; //!< config register object

  /**************************************************************************/
  /*!
      @brief  ctrl_meas register
  */
  /**************************************************************************/
  typedef struct  {
		
		// device mode
    // 00       = sleep
    // 01 or 10 = forced
    // 11       = normal
    unsigned int mode : 2; ///< device mode
    

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_p : 3; ///< pressure oversampling
		
		// temperature oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_t : 3; ///< temperature oversampling

    

    /// @return combined ctrl register
    
  }ctrl_meas;
  extern ctrl_meas _measReg; //!< measurement register object

  /**************************************************************************/
  /*!
      @brief  ctrl_hum register
  */
  /**************************************************************************/
  typedef struct  {
		
		// humidity oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_h : 3; ///< humidity oversampling
    /// unused - don't set
    unsigned int none : 5;

    

  
  }ctrl_hum;
  extern ctrl_hum _humReg; //!< hum register object



  int bme280_init();

  void setSampling(sensor_mode mode,
                   sensor_sampling tempSampling ,
                   sensor_sampling pressSampling,
                   sensor_sampling humSampling,
                   sensor_filter filter,
                   standby_duration duration);

  int takeForcedMeasurement(void);
  float bme280_readTemperature(void);
  float bme280_readPressure(void);
  float bme280_readHumidity(void);

  float readAltitude(float seaLevel);
  float seaLevelForAltitude(float altitude, float pressure);
  uint32_t sensorID(void);

  float getTemperatureCompensation(void);
  void setTemperatureCompensation(float);



  void readCoefficients(void);
  int isReadingCalibration(void);

  void write8(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg);
  uint16_t read16(uint8_t reg);
  uint32_t read24(uint8_t reg);
  int16_t readS16(uint8_t reg);
  uint16_t read16_LE(uint8_t reg); // little endian
  int16_t readS16_LE(uint8_t reg); // little endian

  extern uint8_t _i2caddr;  //!< I2C addr for the TwoWire interface
  extern int32_t _sensorID; //!< ID of the BME Sensor
  extern int32_t t_fine; //!< temperature with high resolution, stored as an attribute
                  //!< as this is used for temperature compensation reading
                  //!< humidity and pressure

  extern int32_t t_fine_adjust; //!< add to compensate temp readings and in turn
                             //!< to pressure and humidity readings

  extern bme280_calib_data _bme280_calib; //!< here calibration data is stored
void bme280_test();
void bme280_task();
#endif
