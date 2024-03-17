/*
MIT License

Copyright (c) 2024 Rémy Hurx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/


#ifndef SCD41_H_
#define SCD41_H_

#include "stdint.h"
#include "stdbool.h"

#define I2C_HANDLER   hi2c1                 // I2C handler, change to the I2C handler
#define I2C_SCD41     0x62

#define SCD41_START_PERIODIC_MEASUREMENT    0x21b1
#define SCD41_READ_MEASUREMENT              0xec05
#define SCD41_STOP_PERIODIC_MEASUREMENT     0x3f86
#define SCD41_SET_TEMPERATURE_OFFSET        0x241d
#define SCD41_GET_TEMPERATURE_OFFSET        0x2318
#define SCD41_SET_SENSOR_ALTITUDE           0x2427
#define SCD41_GET_SENSOR_ALTITUDE           0x2322
#define SCD41_SET_AMBIENT_PRESSURE          0xe000
#define SCD41_GET_AMBIENT_PRESSURE          0xe000
#define SCD41_PERFORM_FORCED_CALIBRATION    0x362f
#define SCD41_SET_AUTOMATIC_SELF_CAL_EN     0x2416
#define SCD41_GET_AUTOMATIC_SELF_CAL_EN     0x2313
#define SCD41_START_LOW_POWER_PERIODIC_M    0x21ac
#define SCD41_GET_DATA_READY_STATUS         0xe4b8
#define SCD41_PERSIST_SETTINGS              0x3615
#define SCD41_GET_SERIAL_NUMBER             0x3682
#define SCD41_PERFORM_SELF_TEST             0x3639
#define SCD41_PERFORM_FACTORY_RESET         0x3632
#define SCD41_REINIT                        0x3646
#define SCD41_MEASURE_SINGLE_SHOT           0x219d
#define SCD41_MEASURE_SINGLE_SHOT_RHT_ONLY  0x2196
#define SCD41_POWER_DOWN                    0x36e0
#define SCD41_WAKE_UP                       0x36f6
#define SCD41_SET_AUTOMATIC_SELF_CAL_PERIOD_INITIAL   0x2445
#define SCD41_SET_AUTOMATIC_SELF_CAL_PERIOD_STANDARD  0x244e
#define SCD41_GET_AUTOMATIC_SELF_CAL_PERIOD_INITIAL   0x2340
#define SCD41_GET_AUTOMATIC_SELF_CAL_PERIOD_STANDARD  0x234b


#define PERSIST                             true
#define NOT_PERSIST                         false


typedef struct {
    /// @brief CO2 level in ppm
    int CO2;
    /// @brief Temperature in degree celsius
    float Temp;
    /// @brief Relative humidity in %RH
    float RH;

  } SensorData;


typedef enum {
  /// @brief Sensor is stopped measuring, idle
  SCD41_STATUS_STOPPED = 0,
  /// @brief Sensor is sleeping
  SCD41_STATUS_SLEEP,
  /// @brief Sensor is measuring periodically, standard
  SCD41_STATUS_PERIODIC_MEASUREMENT,
  /// @brief Sensor is measuring periodically, low power
  SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER,
  /// @brief Sensor is measuring once, with or without CO2
  SCD41_STATUS_ONESHOT_MEASUREMENT,
  /// @brief Sensor is calibrating (Because of long period it has its own status)
  SCD41_STATUS_CALIBRATION_FORCED_ACTIVE

} scd_status_t;


typedef enum {
  SCD41_OK = 0,
  /// @brief Error while transmitting I2C data
  SCD41_ERR_I2C,
  /// @brief Error while calculating CRC
  SCD41_ERR_CRC,
  /// @brief Error while reading sensor data (not I2C)
  SCD41_ERR_MEASUREMENT,
  /// @brief Sensor not ready
  SCD41_ERR_NOT_READY,
  /// @brief Sensor sleeping
  SCD41_ERR_SLEEPING,
  /// @brief Command ignored, like because it was already running
  SCD41_COMMAND_IGNORED,
  /// @brief Sensor malfunction
  SCD41_ERR_MALFUNCTION,
  /// @brief Unknown error/not described
  SCD41_ERR_UNKNOWN
  
} scd41_err_t;


uint8_t calcCrc(uint8_t data[2], int startpoint);
void lightResetScd41();
scd41_err_t singleShotScd41();
scd41_err_t singleShot_NoCO2_Scd41();
scd41_err_t resetScd41();
scd41_err_t persistSetScd41();
scd41_err_t dataReadyScd41();
scd41_err_t readScd41(SensorData *sd);
scd41_err_t startPeriodicMeasurementScd41();
scd41_err_t stopPeriodicMeasurementScd41();
scd41_err_t setTempOffsetScd41(float offset, bool persist);
scd41_err_t getTempOffsetScd41(float *offset);
scd41_err_t setSensorAltitudeScd41(uint16_t height, bool persist);
scd41_err_t getSensorAltitudeScd41(uint16_t *height);
scd41_err_t setAmbientPressureScd41(uint32_t pascal);
scd41_err_t getAmbientPressureScd41(uint32_t *pascal);
scd41_err_t forcedCalScd41(uint16_t targetCO2, uint16_t *FRCCorrection);
scd41_err_t setAutoCalScd41(bool choice, bool persist);
scd41_err_t getAutoCalScd41(bool *status);
scd41_err_t lowPowerMeasurementScd41();
scd41_err_t getSerialScd41(uint64_t *Serial);
scd41_err_t selfTestScd41(bool *statusReceived);
scd41_err_t reInitScd41();
scd41_err_t powerDownScd41();
scd41_err_t wakeUpScd41(bool *verifyWakeUp);
scd41_err_t setAutomaticASCSelfCalPeriod(bool initial, uint16_t ASCHours, bool persist);
scd41_err_t getAutomaticASCSelfCalPeriod(bool initial, uint16_t *ASCHours);


#endif /*SCD41_H_*/