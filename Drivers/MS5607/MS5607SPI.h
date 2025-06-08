/*
   MS5607-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
   05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
   Changelog:
     2012-05-23 - initial release.
*/
/* ============================================================================================
 MS5607-02 device SPI library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2020 João Pedro Vilas Boas

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
 ================================================================================================
 */

#ifndef _MS5607SPI_H_
#define _MS5607SPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32g4xx_hal.h"
#include "ICM42688P/ICM42688PSPI.h"
#include <math.h>
#include <string.h>
#include <global.h>

/* MS5607 SPI COMMANDS */
#define RESET_COMMAND 0x1E
#define PROM_READ(address) (0xA0 | ((address) << 1)) // Macro to change values for the 8 PROM addresses
#define CONVERT_D1_COMMAND 0x40
#define CONVERT_D2_COMMAND 0x50
#define READ_ADC_COMMAND 0x00

  /* MS5607 Oversampling Ratio Enumeration*/
  typedef enum OSRFactors
  {
    OSR_256,
    OSR_512 = 0x02,
    OSR_1024 = 0x04,
    OSR_2048 = 0x06,
    OSR_4096 = 0x08
  } MS5607OSRFactors;

  /* MS5607 System States Enumeration*/
  typedef enum MS5607States
  {
    MS5607_STATE_FAILED,
    MS5607_STATE_READY
  } MS5607StateTypeDef;

  /* Struct to update and return values */
  typedef struct MS5607Readings
  {
    double pressure_kPa;
    double temperature_C;
  } MS5607Readings;

  /**
   * @brief  Initializes MS5607 Sensor
   * @param  SPI Handle address
   * @param  GPIO Port Definition
   * @param  GPIO Pin
   * @retval Initialization status:
   *           - 0 or MS5607_STATE_FAILED: Was not abe to communicate with sensor
   *           - 1 or MS5607_STATE_READY: Sensor initialized OK and ready to use
   */
  MS5607StateTypeDef MS5607_Init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *chip_select_port, uint16_t chip_select_gpio_pin);

  /**
   * @brief  Updates the readings from the sensor
   * @note   This function must be called each time you want new values from the sensor
   * @param  None
   * @retval None
   */
  void MS5607Update(void);
  //
  // /**
  //  * @brief  Gets the temperature reading from the last sensor update
  //  * @note   This function must be called after an @ref MS5607Update() (DOES NOT UPDATE THE READINGS)
  //  * @param  None
  //  * @retval Temperature in celsius
  //  */
  // double MS5607GetLastTemperatureC(void);
  //
  // /**
  //  * @brief  Get pressure in kPa from last sensor update
  //  * @note   This function must be called after an @ref MS5607Update() (DOES NOT UPDATE THE READINGS)
  //  * @param  None
  //  * @retval Pressure in Kilo Pascal
  //  */
  // double MS5607GetLastPressurekPa(void);

  /**
   * @brief  Updates and gets pressure in kPa and temperature in celsius from sensor reading
   * @param  None
   * @retval Pressure in Kilo Pascal
   */
  MS5607Readings MS5607ReadValues(void);

  /**
   * @brief  Enables the chip select pin
   * @param  None
   * @retval None
   */
  void enableCSB(void);

  /**
   * @brief  Disables the chip select pin
   * @param  None
   * @retval None
   */
  void disableCSB(void);

  /**
   * @brief  Sets the temperature reading oversamplig ratio
   * @param  OSR factor from 256 to 4096
   * @retval None
   */
  void MS5607SetTemperatureOSR(MS5607OSRFactors oversampling_ratio);

  /**
   * @brief  Sets the pressure reading oversamplig ratio
   * @param  OSR factor from 256 to 4096
   * @retval None
   */
  void MS5607SetPressureOSR(MS5607OSRFactors oversampling_ratio);

  /*
  Units
  Pressure - mbars - double
  Temperature - Celcius - double
  */

  // https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
  // The altitude equation is for absolute altitude.
  // calibraing : 1 = True, 0 = False

  // pressure should be input in kilopascals!!
  float calculateAltitude(double pressure);
  float calculate_abs_altitude(double pressure);

#ifdef __cplusplus
}
#endif

#endif /* _MS5607SPI_H_ */
