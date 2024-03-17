# Sensirion SCD4X for STM32
---

A simple lib to readout and set options of the Sensirion SCD4X (mostly SCD41). I use it in my projects, first for the
ESP32, but translated it for the STM32 with HAL functions.

Datasheet Sensirion: [datasheet](https://sensirion.com/media/documents/48C4B7FB/6426E14D/CD_DS_SCD40_SCD41_Datasheet_D1_052023.pdf).

## Getting Started

Just add the two files in your project and don't forget to add them to the cmakelists.txt, for example:

```cmake
add_executable(${PROJECT_NAME}        
    Core/Src/Lib/scd41.c
)

include_directories(
    ${PROJECT_SOURCE_DIR}/Core/Inc/Lib
    )
```

Change the I2C handler in the scd41.h file to the one you use

```h
#define I2C_HANDLER   hi2c1                 // I2C handler, change to the I2C handler
```


## Program usage

The lib registers the mode of operation (single shot, continuous, low power continuous), feel free to use or not use in your
project. I start with:

```c
//Resetting by stopping the previous mode of the sensor
lightResetScd41();      
  
//Starting the continuous mode and readout if it was a succes, or not in result
scd41_err_t result = startPeriodicMeasurementScd41();
```

If you want to change some settings like altitude, do this between those two functions.

To readout the sensor:

```c
// Wait for the sensor to be ready (every 5 seconds for the normal mode, 30 seconds low power mode.)
while (dataReadyScd41() != SCD41_OK)
{
    HAL_Delay(500);
}

// Read the sensor data
SensorData readSensorData;

result = readScd41(&readSensorData);
```

## Error Handling

The lib functions typically return an `scd41_err_t` type, which indicates the success or failure of the operation:

- `SCD41_OK`: The operation was successful.
- `SCD41_ERR_I2C`: Error while transmitting I2C data.
- `SCD41_ERR_CRC`: Error while calculating CRC.
- `SCD41_ERR_MEASUREMENT`: Error while reading sensor data (not I2C).
- `SCD41_ERR_NOT_READY`: Sensor not ready.
- `SCD41_ERR_SLEEPING`: Sensor is sleeping.
- `SCD41_COMMAND_IGNORED`: Command ignored, like because it was already running.
- `SCD41_ERR_MALFUNCTION`: Sensor malfunction.
- `SCD41_ERR_UNKNOWN`: Error unknown/not described.


## Contributing

If you have suggestions for improvement, bug reports, or new features, be encouraged to let me know by email
or fork the repository and submit pull requests.


## License

The Sensor Reader Library is released under the MIT License. This license allows you to use, modify, and distribute the library, provided that you include the original copyright notice and disclaimer in any copy of the project or substantial portions of it. For more details, see the [LICENSE](LICENSE) file in the repository.

## Contact

If you have any questions, feedback, or want to discuss the Sensor Reader Library, feel free to reach out. You can contact us via:

- Email: [Rémy Hurx](mailto:github@hurx.nl)
- GitHub Issues: [Project's Issues Page](https://github.com/remyhx/scd41-stm32/issues)




