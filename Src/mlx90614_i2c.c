#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include "mlx90614_i2c.h"

#include "project_config.h"


#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

#if defined(MLX90614)
extern I2C_HandleTypeDef hi2c1;

int mlx90614_init = 0;
long last_milli_celsius;
bool last_milli_celsius_valid = false;
bool want_mlx90614_value;  // set by usb_task

bool has_last_milli_celsius(void)
{
	return last_milli_celsius_valid;
}

long get_last_milli_celsius(void)
{
	return last_milli_celsius; 
}

// cmd: 0x00..0x1f is RAM, 0x20..0x3f is EEPROM. Only these parts can be accessed by this read function.
HAL_StatusTypeDef mlx90614_read_reg(unsigned int cmd, uint16_t* out)
{
	HAL_StatusTypeDef result;

	// We use HAL_I2C_Mem_Read because that seems to be the only way to do it with I2C repeated-start,
    // which is necessary for Melexis MLX90614.
	uint8_t readData[3];
	result = HAL_I2C_Mem_Read(&hi2c1, MLX90614_I2CADDR, cmd, I2C_MEMADD_SIZE_8BIT, readData, 3, MLX90614_COMM_TIMEOUT);
	if (result != HAL_OK)
		return result;

	*out = readData[0] | (readData[1] << 8);
	return HAL_OK;
}

long get_milli_celsius(void)
{
    HAL_StatusTypeDef result;

    uint16_t value;
    result = mlx90614_read_reg(MLX90614_TOBJ, &value);
    if (result != HAL_OK)
	{
		last_milli_celsius_valid = false;
        return MLX90614_INVALID_MILLI_CELSIUS;
	}

    if (value & 0x8000)
	{
        // sensor tells us that the value isn't valid
		last_milli_celsius_valid = false;
        return MLX90614_INVALID_MILLI_CELSIUS;
	}

    //float kelvin = value * 0.020;
	//float celsius = kelvin - 273.150;

	long milli_kelvin = value * 20;
	long milli_celsius = milli_kelvin - 273150;

	last_milli_celsius = milli_celsius;
	last_milli_celsius_valid = true;

    return milli_celsius;
}

long mlx90614_read_ambient_temperature(void)
{
    HAL_StatusTypeDef result;

    uint16_t value;
    result = mlx90614_read_reg(MLX90614_TAMBIENT, &value);
    if (result != HAL_OK)
        return MLX90614_INVALID_MILLI_CELSIUS;

    if (value & 0x8000)
	{
        // sensor tells us that the value isn't valid
        return MLX90614_INVALID_MILLI_CELSIUS;
	}

	long milli_kelvin = value * 20;
	long milli_celsius = milli_kelvin - 273150;

    return milli_celsius;
}

int convert_C_to_F(int C)
{
	return ((C*18 + 320)/10);
}

int read_mlx90614_regs(bool force)
{
	long temperature;

	if (!want_mlx90614_value && !force)
		return 0;

	if(mlx90614_init == 0)
	{
		HAL_StatusTypeDef result;

		last_milli_celsius_valid = false;

		// Melexis hasn't documented any way of detecting an MLX90614 (or even to tell apart which type it is).
		// Therefore, we look at some default values in EEPROM. There is no reason to change them when using
		// the sensor in I2C mode so we assume that users won't change them. These values seem to be the same
		// even non-original devices that differ in other regards, namely how they calculate the checksum.
		uint16_t eeprom0, eeprom1;
		result = mlx90614_read_reg(MLX90614_EEPROM0, &eeprom0);
		if (result != HAL_OK) {
			// No reply from sensor so we assume that no sensor is connected. We keep mlx90614_init set to zero
			// so we will try again.
			return 0;
		}

		// If we fail after this point, don't try again.
		mlx90614_init = -1;

		result = mlx90614_read_reg(MLX90614_EEPROM1, &eeprom1);
		if (result != HAL_OK)
			return 0;
		if (eeprom0 != 0x9993 || eeprom1 != 0x62e3)
		{
			DEBUG_PRINTF("We found some device at I2C address 0x5a but we got unexpected values when reading EEPROM cells 0 and 1. Therefore, we assume that it is not an MLX90614 sensor.\n");
			return 0;
		}

		long ambientTemperatureCelsius = mlx90614_read_ambient_temperature() / 1000;
		long objectTemperatureCelsius = get_milli_celsius() / 1000;
		last_milli_celsius_valid = false;  // might be set by get_milli_celsius() but we don't accept the value, yet

		//NOTE Both ranges are a bit wider than what is supported according to the datasheet.
		//     (If we couldn't read the temperatures, the value will be -420, which will also fail the checks.)
		if (ambientTemperatureCelsius < -60 || ambientTemperatureCelsius > 150 || objectTemperatureCelsius < -100 || objectTemperatureCelsius > 500)
		{
			DEBUG_PRINTF("We got unexpected temperatures from MLX90614: ambient %d °C, object %d °C\n", ambientTemperatureCelsius, objectTemperatureCelsius);
			return 0;
		}

		DEBUG_PRINTF("We found an MLX90614 connected to the PureThermal board. Current temperatures are %d °C (ambient, i.e. the sensor itself) and %d °C (object).\n",
			ambientTemperatureCelsius, objectTemperatureCelsius);

		mlx90614_init = 1;
	}

	if (mlx90614_init == 1)
	{
		temperature = get_milli_celsius();
		(void) temperature; // avoid warning
		DEBUG_PRINTF("MLX90614_TOBJ: %x  mC: %ld  C: %ld  F: %d\n\r",mlx90614_read_reg(MLX90614_TOBJ)>>2,temperature,temperature/1000, convert_C_to_F(temperature/1000));
	}

	return last_milli_celsius_valid;
}

#endif