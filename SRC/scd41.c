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

Be careful with the scd41 instructions to stop almost always periodic measurement!
Some delays are neccessary for the instructions to work, change to osdelay for rtos.
Some texts are copied from the datasheet of the sensor, to make it easier to understand the functions.

*/

#include "main.h"
#include "scd41.h"
#include <stdio.h>
#include "math.h"

extern I2C_HandleTypeDef I2C_HANDLER;                   //Change to I2C handler


HAL_StatusTypeDef status;
scd_status_t      scdStatus;

uint8_t           data[25];


/*!
 * @brief Calculates the CRC to be checked by the sensor.
 *
 * This function calculates the CRC (Cyclic Redundancy Check) for a two-byte data array.
 *
 * @param data The two-byte data array.
 * @param startpoint The start point of the array, if a bigger byte-array is used.
 * @return The calculated CRC value.
 */
uint8_t calcCrc(uint8_t data[2], int startpoint)
{
  uint8_t crc = 0xFF;
  for (int i = startpoint; i < (startpoint+2); i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 8; bit > 0; --bit)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x31u;
        
      }
      else
      {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}


/*!
* @brief If the sensor is running in periodic measurement mode it can mess up a lot of the commands! 
Especially when it runs for the first time, after running it knows the status of the SCD41 by the scd_status_t
status. I recommend starting with this option. There is no good check if the periodic is running..
* @return nothing
*/
void lightResetScd41()
{
  stopPeriodicMeasurementScd41();
}


/*!
 * @brief On-demand measurement of CO2 concentration, relative humidity and temperature. Keep in mind to perform this function several times after a long delay, to make the readout accurate.
 *
 * @return returns the status, SCD41_OK if succeeded, other if not.
 */
scd41_err_t singleShotScd41()
{
  // Readout with read measurement command (& ready flag), takes 5 sec to initialise
  // Beware: singleshot doesnt stop periodic measurement! Only after a stop command and singleshot is per shot.
  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  data[0]=SCD41_MEASURE_SINGLE_SHOT>>8;
  data[1]=SCD41_MEASURE_SINGLE_SHOT&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  
  if (status == HAL_OK) {
    scdStatus = SCD41_STATUS_ONESHOT_MEASUREMENT;
    return SCD41_OK;
   } else return SCD41_ERR_I2C;
}


/*!
 * @brief      Same as singleShotScd41, except CO2 isn't measured.
 *
 * @return     returns the status, SCD41_OK if succeeded, other if not.
 */
scd41_err_t singleShot_NoCO2_Scd41()
{
  // Readout without CO2
  // Beware: singleshot doesnt stop periodic measurement! Only after a stop command and singleshot is per shot.
  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  data[0]=SCD41_MEASURE_SINGLE_SHOT_RHT_ONLY>>8;
  data[1]=SCD41_MEASURE_SINGLE_SHOT_RHT_ONLY&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);

  if (status == HAL_OK) {
    scdStatus = SCD41_STATUS_ONESHOT_MEASUREMENT;
    return SCD41_OK; 
  }else return SCD41_ERR_I2C;
}


/*!
 * @brief      The perform_factory_reset command resets all configuration settings stored in the EEPROM and erases the
FRC and ASC algorithm history. 
 *
 * @return     returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t resetScd41()
{
  // Factory reset
  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  
  data[0]=SCD41_PERFORM_FACTORY_RESET>>8;
  data[1]=SCD41_PERFORM_FACTORY_RESET&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  
  if (status == HAL_OK) return SCD41_OK; else return SCD41_ERR_I2C;
}


/*!
 * @brief     Configuration settings such as the temperature offset, sensor altitude and the ASC enabled/disabled parameter
are by default stored in the volatile memory (RAM) only and will be lost after a power-cycle. The persist_settings command
stores the current configuration in the EEPROM of the SCD4x, making them persistent across power-cycling. To avoid
unnecessary wear of the EEPROM, the persist_settings command should only be sent when persistence is required and if actual
changes to the configuration have been made. The EEPROM is guaranteed to endure at least 2000 write cycles before failure.
Note that field calibration history (i.e. FRC and ASC, see chapter 3.7) is automatically stored in a separate EEPROM
dimensioned for the specified sensor lifetime. 
 *
 * @return     returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t persistSetScd41()
{
  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  // Write in non-volatile memory
  data[0]=SCD41_PERSIST_SETTINGS>>8;
  data[1]=SCD41_PERSIST_SETTINGS&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);

  HAL_Delay(1000);

  if (status == HAL_OK) return SCD41_OK; else return SCD41_ERR_I2C;
} 


/*!
 * @brief      Checks if the sensor is ready for readout gathered data.
 * 
 * @return     TRUE if data is ready to be read, FALSE if not.
 */
scd41_err_t dataReadyScd41()
{
  // Readout data ready flag
  uint16_t message;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  
  data[0]=SCD41_GET_DATA_READY_STATUS>>8;
  data[1]=SCD41_GET_DATA_READY_STATUS&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(100);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  //Process data
  //CRC check first
  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_I2C; //CRC didn't match!
  message = data[0]<<8 | data[1];
  message = message & 0b0000011111111111;   //LSB 11 bits shouldn't be 0
  
  if (message==0) return SCD41_ERR_NOT_READY; else return SCD41_OK;
}


/*!
 * @brief      Read sensor output. The measurement data can only be read out once per signal update interval as the buffer is
emptied upon read-out. If no data is available in the buffer, the sensor returns a NACK. To avoid a NACK response, this functions checks the data ready flag before.
 *
 * @return     NULL
 */
scd41_err_t readScd41(SensorData *sd)
{
  uint16_t message;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;

  data[0] = SCD41_READ_MEASUREMENT>>8;
  data[1] = SCD41_READ_MEASUREMENT&0x00FF;

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;
  
  HAL_Delay(100);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 9, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  //CRC check
  for (int i=0; i<3; i++) {
  if (calcCrc(&data[3*i],0)!=data[3*i+2]) return SCD41_ERR_CRC; //CRC didn't match!
  }
  message = data[0] << 8 | data[1];
  sd->CO2 = message;

  message = data[3] << 8 | data[4];
  sd->Temp = ((message*175.00)/65536) - 45;

  message = data[6] << 8 | data[7];
  sd->RH = ((message*100.00)/65536);

  if (scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) scdStatus = SCD41_STATUS_STOPPED;

  return SCD41_OK;
}


/*!
 * @brief      start periodic measurement, signal update interval is 5 seconds.
 *
 * @return     returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t startPeriodicMeasurementScd41()
{
  //Signal update 5 seconds
  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) return SCD41_COMMAND_IGNORED;

  data[0]=SCD41_START_PERIODIC_MEASUREMENT>>8;
  data[1]=SCD41_START_PERIODIC_MEASUREMENT&0x00FF;

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(500);

  scdStatus = SCD41_STATUS_PERIODIC_MEASUREMENT;
  return SCD41_OK;
}


/*!
 * @brief      Stop periodic measurement to change the sensor configuration or to save power.
 *
 * @return     returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t stopPeriodicMeasurementScd41()
{
  //Stop periodic measurement
  
  data[0]=SCD41_STOP_PERIODIC_MEASUREMENT>>8;
  data[1]=SCD41_STOP_PERIODIC_MEASUREMENT&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(500);

  scdStatus = SCD41_STATUS_STOPPED;

  return SCD41_OK;
}


/*!
 * @brief       The temperature offset has no influence on the SCD4x CO2 accuracy. Setting the temperature offset of the SCD4x
inside the customer device correctly allows the user to leverage the RH and T output signal. Note that the temperature offset
can depend on various factors such as the SCD4x measurement mode, self-heating of close components, the ambient
temperature and air flow. Thus, the SCD4x temperature offset should be determined inside the customer device under its typical
operation conditions (including the operation mode to be used in the application) and in thermal equilibrium. Per default, the
temperature offset is set to 4° C. To save the setting to the EEPROM, the persist setting (see chapter 3.9.1) command must be
issued. Equation (1) shows how the characteristic temperature offset can be obtained.
𝑇𝑜𝑓𝑓𝑠𝑒𝑡_𝑎𝑐𝑡𝑢𝑎𝑙 = 𝑇𝑆𝐶𝐷40 − 𝑇𝑅𝑒𝑓𝑒𝑟𝑒𝑛𝑐𝑒 + 𝑇𝑜𝑓𝑓𝑠𝑒𝑡_ 𝑝𝑟𝑒𝑣𝑖𝑜𝑢𝑠 
 *
 * @param[in] offset Offset to be ommited.
 * @param[in] persist true if needs to be written to eeprom, otherwise false
 * 
 * @return     returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t setTempOffsetScd41(float offset, bool persist)
{
  uint16_t message;
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  
  data[0] = SCD41_SET_TEMPERATURE_OFFSET>>8;
  data[1] = SCD41_SET_TEMPERATURE_OFFSET&0x00FF;

  if ((offset < 0) || (offset>65535)) return SCD41_ERR_UNKNOWN;
  
  message = ((offset*65535.0)/175);

  data[2] = message>>8;
  data[3] = message&0x00FF;
  data[4] = calcCrc(&data[2],0);

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 5, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (persist == PERSIST) persistSetScd41();

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
 * @brief Get temperature offset.
 *
 * @param offset will return offset that is registered.
 *
 * @return returns the status, scd41_OK if succeeded, other if not. 
 */
scd41_err_t getTempOffsetScd41(float *offset)
{
  uint16_t message;
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  
  data[0] = SCD41_GET_TEMPERATURE_OFFSET>>8;
  data[1] = SCD41_GET_TEMPERATURE_OFFSET&0x00FF; 

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;
  
  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_CRC;  
  
  message = data[0]<<8 | data[1];

  float tempValue = ((message*175.0)/65535);
  
  *offset = (roundf(tempValue * 10)) / 10;

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
 * @brief     Reading and writing of the sensor altitude must be done while the SCD4x is in idle mode. Typically, the sensor
altitude is set once after device installation. To save the setting to the EEPROM, the persist setting command
must be issued. Per default, the sensor altitude is set to 0 meter above sea-level.

 *
 * @param[in] height Hight in meters (sensor altitude)
 * @param[in] persist True: write altitude to EEPROM. False to not-persist.
 * 
 * @return returns the status, scd41_OK if succeeded, other if not. 
 */
scd41_err_t setSensorAltitudeScd41(uint16_t height, bool persist)
{
  uint16_t message;
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  
  data[0] = SCD41_SET_SENSOR_ALTITUDE>>8;
  data[1] = SCD41_SET_SENSOR_ALTITUDE&0x00FF;
  
  message = height;

  data[2] = message>>8;
  data[3] = message&0x00FF;
  data[4] = calcCrc(&data[2],0);

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 5, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (persist == PERSIST) persistSetScd41();

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
 * @brief Get sensor altitude in meters.
 * 
 * @param height height like put in the sensor.
 * 
 * @return returns the status, scd41_OK if succeeded, other if not. 
 */
scd41_err_t getSensorAltitudeScd41(uint16_t *height)
{
  uint16_t message;
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  
  data[0] = SCD41_GET_SENSOR_ALTITUDE>>8;
  data[1] = SCD41_GET_SENSOR_ALTITUDE&0x00FF; 
 
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_CRC;  
  
  *height = data[0]<<8 | data[1];
  
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
 * @brief      The set_ambient_pressure command can be sent during periodic measurements to enable continuous pressure
compensation. Note that setting an ambient pressure using set_ambient_pressure overrides any pressure compensation based
on a previously set sensor altitude.
 *
 * @param[in] pascal Pressure in pascal (UINT32_T), intended for continues pressure compensation. For example 1001hPa is 100100!
 * 
 * @return returns the status, scd41_OK if succeeded, other if not. 
 */
scd41_err_t setAmbientPressureScd41(uint32_t pascal)
{
  uint16_t message;

    scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  
  if ((pascal < 0) || (pascal > 6553500)) return SCD41_ERR_UNKNOWN;

  message = (uint16_t)(pascal/100); //like 98700 pa

  data[0] = SCD41_SET_AMBIENT_PRESSURE>>8;
  data[1] = SCD41_SET_AMBIENT_PRESSURE&0x00FF;

  data[2] = message>>8;
  data[3] = message&0x00FF;
  data[4] = calcCrc(&data[2],0);

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 5, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  return SCD41_OK;
}


/*!
 * @brief      The get_ambient_pressure command can be sent during periodic measurements to retrieve the pressure
compensation. 
 *
 * @param[in] pascal Pressure in pascal (UINT32_T), as set in the sensor.
 * 
 * @return returns the status, scd41_OK if succeeded, other if not. 
 */
scd41_err_t getAmbientPressureScd41(uint32_t *pascal)
{
  uint16_t message;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;

  data[0] = SCD41_GET_AMBIENT_PRESSURE>>8;
  data[1] = SCD41_GET_AMBIENT_PRESSURE&0x00FF;

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(100);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_CRC;  
  
  *pascal = (data[0]<<8 | data[1])*100;

  return SCD41_OK;
}


/*!
 * @brief      Set forced calibration for SCD41. The procedure is to operate the SCD41 for more then 3 minutes in the desired operating mode (periodic, low power periodic or single shot) and preferably outside (or known CO2 level environment) 
 *
 * @param[in] targetCO2 The target CO2 level to correct to.
 * 
 * @return     returns the status, scd41_OK if succeeded, other if not.
 
 */
scd41_err_t forcedCalScd41(uint16_t targetCO2, uint16_t *FRCCorrection)
{
  uint16_t message;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;

  //First operate sensor 3 minutes in periodic measurement mode, then stop and perform this function.
  if (startPeriodicMeasurementScd41() != SCD41_OK) {

    //When start periodic measurement fails, try to stop it and start again (mostly because it was already running)
    stopPeriodicMeasurementScd41();
    HAL_Delay(1000);
    scd41_err_t polScd =  startPeriodicMeasurementScd41();
    if (polScd != SCD41_OK) return SCD41_ERR_UNKNOWN;
  }

  HAL_Delay(180000); //3 minutes (180000ms)

  //Stop periodic measurement
  if (stopPeriodicMeasurementScd41() != SCD41_OK) return SCD41_ERR_UNKNOWN;
  scdStatus = SCD41_STATUS_STOPPED;

  //Mandatory delay
  HAL_Delay(500);

  //Continue with forced calibration

  data[0] = SCD41_PERFORM_FORCED_CALIBRATION>>8;
  data[1] = SCD41_PERFORM_FORCED_CALIBRATION&0x00FF;
  
  if (targetCO2 < 400 || targetCO2 > 5000) return SCD41_ERR_UNKNOWN;

  message = targetCO2;

  data[2] = message>>8;
  data[3] = message&0x00FF;
  data[4] = calcCrc(&data[2],0);

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 5, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(1000);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_CRC;

  message = data[0]<<8 | data[1];

  if (message == 0xffff) return SCD41_ERR_MEASUREMENT; //No valid FRC correction value available / error
  *FRCCorrection = message - 0x8000;
  
  return SCD41_OK;
}


/*!
 * @brief      Set the current state (enabled / disabled) of the automatic self-calibration. By default, ASC is enabled. To save the
setting to the EEPROM, the persist_setting command must be issued.
 *
 * @param[in] choice true for ASC.
 * @param[in] persist true for persist.
 * 
 * @return returns the status, scd41_OK if succeeded, other if not. 
 */
scd41_err_t setAutoCalScd41(bool choice, bool persist)
{
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  data[0] = SCD41_SET_AUTOMATIC_SELF_CAL_EN>>8;
  data[1] = SCD41_SET_AUTOMATIC_SELF_CAL_EN&0x00FF;
  
  if (choice == true)
  {
  data[2] = 0x00;
  data[3] = 0x01;
  data[4] = 0xb0;
  }
  else 
  {
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x81;
  }

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 5, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (persist == PERSIST) persistSetScd41();

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
 * @brief Gets the ASC setting
 *
 * @param[out] statusASC true if ASC is enabled.
 * 
 * @return returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t getAutoCalScd41(bool *statusASC)
{
  uint16_t message;
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  

  data[0] = SCD41_GET_AUTOMATIC_SELF_CAL_EN>>8;
  data[1] = SCD41_GET_AUTOMATIC_SELF_CAL_EN&0x00FF;

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(100);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_CRC;
  message = data[0]<<8 | data[1];

  if (message == 0x0001) *statusASC = true;
  if (message == 0x0000) *statusASC = false;
  if (message != 0x0001 && message != 0x0000) return SCD41_ERR_UNKNOWN;

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
 * @brief start low power periodic measurement, signal update interval is approximately 30 seconds.
 *
 * @return returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t lowPowerMeasurementScd41()
{
  //Signal update 5 seconds
  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER  || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  
  data[0]=SCD41_START_LOW_POWER_PERIODIC_M>>8;
  data[1]=SCD41_START_LOW_POWER_PERIODIC_M&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(500);

  scdStatus = SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER;

  return SCD41_OK;
}


/*!
 * @brief      Reading out the serial number can be used to identify the chip and to verify the presence of the sensor.
 *
 * @return     NULL by function, sets serial in Scd41_Serial struct.
 */
scd41_err_t getSerialScd41(uint64_t *Serial)
{
  uint16_t message1, message2, message3; 
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  data[0] = SCD41_GET_SERIAL_NUMBER>>8;
  data[1] = SCD41_GET_SERIAL_NUMBER&0x00FF;

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(100);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 9, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;


  for (int i=0; i<3; i++){
  if (calcCrc(&data[i*3],0)!=data[2+3*i]) return SCD41_ERR_CRC; //CRC didn't match!
  }

  message1 = data[0] << 8 | data[1];
  message2 = data[3] << 8 | data[4];
  message3 = data[6] << 8 | data[7];

  *Serial = (uint64_t)message1 << 32 | (uint64_t)message2 << 16 | (uint64_t)message3;
  
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
 * @brief      The self test feature can be used as an end-of-line test to check sensor functionality and the customer
power supply to the sensor. 
*
* @param[out] statusReceived true if self test succeeded.
 *
 * @return returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t selfTestScd41(bool *statusReceived)
{
  uint16_t message;
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  data[0] = SCD41_PERFORM_SELF_TEST>>8;
  data[1] = SCD41_PERFORM_SELF_TEST&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(10000);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  message = data[0]<<8|data[1];
  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_CRC; //CRC didn't match!

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  if (message == 0) {
    *statusReceived = true;
    return SCD41_OK;
  }
  else {
    *statusReceived = false;
    return SCD41_ERR_MALFUNCTION;
  }

}


/*!
 * @brief      The reinit command reinitializes the sensor by reloading user settings from EEPROM. If the reinit command does not trigger the desired re-initialization,
a power-cycle should be applied to the SCD4x. 

 *
 * @return returns the status, scd41_OK if succeeded, other if not.
 */
scd41_err_t reInitScd41()
{
  //Get back to user settings, has to be stopped taking measurements
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  data[0] = SCD41_REINIT>>8;
  data[1] = SCD41_REINIT&0x00FF;

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(1000);

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


scd41_err_t powerDownScd41()
{
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }
  
  data[0]=SCD41_POWER_DOWN>>8;
  data[1]=SCD41_POWER_DOWN&0x00FF;
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  scdStatus = SCD41_STATUS_SLEEP;
  return SCD41_OK;
}


scd41_err_t wakeUpScd41(bool *verifyWakeUp)
{
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    return SCD41_ERR_UNKNOWN;
  }

  data[0]=SCD41_WAKE_UP>>8;
  data[1]=SCD41_WAKE_UP&0x00FF;

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  //Sensor will not aknowledge the wake up command, so it will return HAL_ERROR, but it will wake up.
  //Therefore no status processing

  HAL_Delay(100);

  //Verify if neccessary
  if (*verifyWakeUp == true) {
    uint64_t scd41Serial;
    scd41_err_t tempStatus = getSerialScd41(&scd41Serial);
    if (tempStatus != SCD41_OK) *verifyWakeUp = true; else *verifyWakeUp = false;
  }

  scdStatus = SCD41_STATUS_STOPPED;

  return SCD41_OK;

}



/*!
* @brief Set the initial period for ASC correction (in hours). By default, the initial period for 
ASC correction is 44 hours. Allowed values are integer multiples of 4 hours. (If not will be auto 
corrected up) Note: Assumes an average measurement interval of 5 minutes in single shot operation. For 
different average single shot measurement intervals, the parameter value should be scaled inversely 
proportional to this (e.g. by factor 0.5 for 10 minutes average single shot interval). Note: a value of 
0 results in an immediate correction. To save the setting to the EEPROM, the persist_settings (see 
Section 3.9.1) command must be issued.
* @param initial If for initial period true and for standard period false
* @param ASCHours Preferably a multiple of 4, as mentioned in the datasheet
* @param persist True if needs to be written to eeprom, otherwise false
* @return returns the status, scd41_OK if succeeded, other if not.
*/
scd41_err_t setAutomaticASCSelfCalPeriod(bool initial, uint16_t ASCHours, bool persist)
{
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  //Correct the ASCHours to be a multiple of 4, as mentioned in the datasheet
  uint16_t correctedASCHours;
  uint16_t rest = ASCHours % 4;
  if (rest == 0) {
      correctedASCHours = ASCHours;
  } else {
      correctedASCHours = ASCHours + (4 - rest);
  }

  if (initial == true) {
    data[0] = SCD41_SET_AUTOMATIC_SELF_CAL_PERIOD_INITIAL>>8;
    data[1] = SCD41_SET_AUTOMATIC_SELF_CAL_PERIOD_INITIAL&0x00FF;
  } else {
    data[0] = SCD41_SET_AUTOMATIC_SELF_CAL_PERIOD_STANDARD>>8;
    data[1] = SCD41_SET_AUTOMATIC_SELF_CAL_PERIOD_STANDARD&0x00FF;
  }

  data[2] = correctedASCHours>>8;
  data[3] = correctedASCHours&0x00FF;
  data[4] = calcCrc(&data[2],0);
  

  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 5, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (persist == PERSIST) persistSetScd41();

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}


/*!
* @brief Get the initial/standard period for ASC correction (in hours).
*
* @param initial If for initial period true and for standard period false
* @param[out] ASCHours The set hours for the initial/standard ASC
*
* @return returns the status, scd41_OK if succeeded, other if not.
*/
scd41_err_t getAutomaticASCSelfCalPeriod(bool initial, uint16_t *ASCHours)
{
  scd_status_t backupStatus;

  if (scdStatus == SCD41_STATUS_SLEEP) return SCD41_ERR_SLEEPING;
  if (scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT || scdStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER || scdStatus == SCD41_STATUS_ONESHOT_MEASUREMENT)
  {
    backupStatus = scdStatus;
    stopPeriodicMeasurementScd41();
    scdStatus = SCD41_STATUS_STOPPED;
  }

  if (initial == true) {
    data[0] = SCD41_GET_AUTOMATIC_SELF_CAL_PERIOD_INITIAL>>8;
    data[1] = SCD41_GET_AUTOMATIC_SELF_CAL_PERIOD_INITIAL&0x00FF;
  } else {
    data[0] = SCD41_GET_AUTOMATIC_SELF_CAL_PERIOD_STANDARD>>8;
    data[1] = SCD41_GET_AUTOMATIC_SELF_CAL_PERIOD_STANDARD&0x00FF;
  }
  
  status = HAL_I2C_Master_Transmit(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 2, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  HAL_Delay(300);

  status = HAL_I2C_Master_Receive(&I2C_HANDLER, (uint16_t)(I2C_SCD41 << 1), data, 3, HAL_MAX_DELAY);
  if (status != HAL_OK) return SCD41_ERR_I2C;

  if (calcCrc(&data[0],0)!=data[2]) return SCD41_ERR_CRC;

  *ASCHours = data[0]<<8 | data[1];

  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT) startPeriodicMeasurementScd41();
  if (backupStatus == SCD41_STATUS_PERIODIC_MEASUREMENT_LOWPOWER) lowPowerMeasurementScd41();
  if (backupStatus == SCD41_STATUS_ONESHOT_MEASUREMENT) singleShotScd41();

  return SCD41_OK;
}