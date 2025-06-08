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

#include "MS5607SPI.h"


/* MS5607 PROM Data Structure */
struct promData
{
  uint16_t reserved;
  uint16_t sens;
  uint16_t off;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t tempsens;
  uint16_t crc;
};

/* Uncompensated digital Values */
struct MS5607UncompensatedValues
{
  uint32_t pressure;
  uint32_t temperature;
};

/* Actual readings */
struct MS5607IntReadings
{
  int32_t pressure;
  int32_t temperature;
};

/**
 * @brief  Reads MS5607 PROM Content
 * @note   Must be called only on device initialization
 * @param  Address of promData structure instance
 * @retval None
 */
void MS5607PromRead(struct promData *);

/**
 * @brief  Reads uncompensated content from the MS5607 ADC
 * @note   Must be called before every convertion
 * @param  Address of MS5607 UncompensatedValues Structure
 * @retval None
 */
void MS5607UncompensatedRead(struct MS5607UncompensatedValues *);

/**
 * @brief  Converts uncompensated values into real world values using data from @ref promData and @ref MS5607UncompensatedValues
 * @note   Must be called after @ref MS5607UncompensatedRead
 * @param  Address of MS5607UncompensatedValues Structure
 * @param  Address of MS5607Readings Structure
 * @retval None
 */
void MS5607Convert(struct MS5607UncompensatedValues *, struct MS5607IntReadings *);

/* Private SPI Handler */
static SPI_HandleTypeDef *hspi;

/* Private GPIO CS Pin Variables */
static GPIO_TypeDef *ChipSelect_GPIO_Port;
static uint16_t ChipSelect_Pin;

/* SPI Transmission Data */
static uint8_t SPITransmitData;

/* Private OSR Instantiations */
static uint8_t Pressure_OSR = OSR_256;
static uint8_t Temperature_OSR = OSR_256;

/* Private data holders */

/* PROM data structure */
static struct promData promData;
/* Uncompensated values structure */
static struct MS5607UncompensatedValues uncompValues;
/* Compensated values structure */
static struct MS5607IntReadings readings;

/** Reset and prepare for general usage.
 * This will reset the device and perform the PROM reading to find the conversion values and if
 * the communication is working.
 */
MS5607StateTypeDef MS5607_Init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *chip_select_port, uint16_t chip_select_gpio_pin)
{
  hspi = spi_handle;
  ChipSelect_GPIO_Port = chip_select_port;
  ChipSelect_Pin = chip_select_gpio_pin;

  enableCSB();
  SPITransmitData = RESET_COMMAND;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  HAL_Delay(3);
  disableCSB();

  MS5607PromRead(&promData);

  if (promData.reserved == 0x00 || promData.reserved == 0xff)
    return MS5607_STATE_FAILED;
  else
    return MS5607_STATE_READY;
}

/* Performs a reading on the devices PROM. */
void MS5607PromRead(struct promData *prom)
{
  uint8_t address;
  uint16_t *structPointer;

  /* As the PROM is made of 8 16bit addresses I used a pointer for accessing the data structure */
  structPointer = (uint16_t *)prom;

  for (address = 0; address < 8; address++)
  {
    SPITransmitData = PROM_READ(address);
    enableCSB();
    HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
    /* Receive two bytes at once and stores it directly at the structure */
    HAL_SPI_Receive(hspi, structPointer, 2, 10);
    disableCSB();
    structPointer++;
  }

  /* Byte swap on 16bit integers*/
  structPointer = (uint16_t *)prom;
  for (address = 0; address < 8; address++)
  {
    uint8_t *toSwap = (uint8_t *)structPointer;
    uint8_t secondByte = toSwap[0];
    toSwap[0] = toSwap[1];
    toSwap[1] = secondByte;
    structPointer++;
  }
}

/* Performs a reading on the devices PROM. */
void MS5607UncompensatedRead(struct MS5607UncompensatedValues *uncompValues)
{

  /*Sensor reply data buffer*/
  uint8_t reply[3];

  enableCSB();
  /* Assemble the conversion command based on previously set OSR */
  SPITransmitData = CONVERT_D1_COMMAND | Pressure_OSR;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);

  if (Pressure_OSR == 0x00)
    HAL_Delay(1);
  else if (Pressure_OSR == 0x02)
    HAL_Delay(2);
  else if (Pressure_OSR == 0x04)
    HAL_Delay(3);
  else if (Pressure_OSR == 0x06)
    HAL_Delay(5);
  else
    HAL_Delay(10);

  disableCSB();

  /* Performs the reading of the 24 bits from the ADC */

  enableCSB();

  SPITransmitData = READ_ADC_COMMAND;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  HAL_SPI_Receive(hspi, reply, 3, 10);

  disableCSB();

  /* Tranfer the 24bits read into a 32bit int */
  uncompValues->pressure = ((uint32_t)reply[0] << 16) | ((uint32_t)reply[1] << 8) | (uint32_t)reply[2];

  enableCSB();

  /* Assemble the conversion command based on previously set OSR */
  SPITransmitData = CONVERT_D2_COMMAND | Temperature_OSR;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);

  if (Temperature_OSR == 0x00)
    HAL_Delay(1);
  else if (Temperature_OSR == 0x02)
    HAL_Delay(2);
  else if (Temperature_OSR == 0x04)
    HAL_Delay(3);
  else if (Temperature_OSR == 0x06)
    HAL_Delay(5);
  else
    HAL_Delay(10);

  disableCSB();

  enableCSB();

  SPITransmitData = READ_ADC_COMMAND;
  HAL_SPI_Transmit(hspi, &SPITransmitData, 1, 10);
  HAL_SPI_Receive(hspi, reply, 3, 10);

  disableCSB();

  /* Assemble the conversion command based on previously set OSR */
  uncompValues->temperature = ((uint32_t)reply[0] << 16) | ((uint32_t)reply[1] << 8) | (uint32_t)reply[2];
}

/* Performs the data conversion according to the MS5607 datasheet */
void MS5607Convert(struct MS5607UncompensatedValues *sample, struct MS5607IntReadings *value)
{
  int32_t dT;
  int32_t TEMP;
  int64_t OFF;
  int64_t SENS;

  dT = sample->temperature - ((int32_t)(promData.tref << 8));

  TEMP = 2000 + (((int64_t)dT * promData.tempsens) >> 23);

  OFF = ((int64_t)promData.off << 17) + (((int64_t)promData.tco * dT) >> 6);
  SENS = ((int64_t)promData.sens << 16) + (((int64_t)promData.tcs * dT) >> 7);

  /**/
  if (TEMP < 2000)
  {
    int32_t T2 = ((int64_t)dT * (int64_t)dT) >> 31;
    int32_t TEMPM = TEMP - 2000;
    int64_t OFF2 = (61 * (int64_t)TEMPM * (int64_t)TEMPM) >> 4;
    int64_t SENS2 = 2 * (int64_t)TEMPM * (int64_t)TEMPM;
    if (TEMP < -1500)
    {
      int32_t TEMPP = TEMP + 1500;
      int32_t TEMPP2 = TEMPP * TEMPP;
      OFF2 = OFF2 + (int64_t)15 * TEMPP2;
      SENS2 = SENS2 + (int64_t)8 * TEMPP2;
    }
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
  }

  value->pressure = ((((int64_t)sample->pressure * SENS) >> 21) - OFF) >> 15;
  value->temperature = TEMP;
}

/* Performs the sensor reading updating the data structures */
void MS5607Update(void)
{
  MS5607UncompensatedRead(&uncompValues);
  MS5607Convert(&uncompValues, &readings);
}

MS5607Readings MS5607ReadValues(void)
{
  MS5607Update();
  return (MS5607Readings){.pressure_kPa = readings.pressure / 1000.0, .temperature_C = readings.temperature / 100.0};
}

// double MS5607GetTempC(void) {
//	MS5607Update();
//	return MS5607GetLastTemperatureC();
// }
//
// double MS5607GetPressurekPa(void) {
//	MS5607Update();
//	return MS5607GetLastPressurekPa();
// }

/* Gets the temperature from the sensor reading */
double MS5607GetLastTemperatureC(void)
{
  return (double)readings.temperature / (double)100.0;
}

/* Gets the pressure from the sensor reading */
double MS5607GetLastPressurekPa(void)
{
  return readings.pressure / 1000.0;
}

/* Sets the CS pin */
void enableCSB(void)
{
  HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_RESET);
}

/* Sets the CS pin */
void disableCSB(void)
{
  HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_SET);
}

/* Sets the OSR for temperature */
void MS5607SetTemperatureOSR(MS5607OSRFactors tOSR)
{
  Temperature_OSR = tOSR;
}

/* Sets the OSR for pressure */
void MS5607SetPressureOSR(MS5607OSRFactors pOSR)
{
  Pressure_OSR = pOSR;
}

/*
  AltitudeCalculations.c START HERE
*/
float const lower_altitude_threshold = 20.0;

// Index is by seconds ago the value was calculated
float altitude_history[] = {0, 0, 0};

float max_altitude = 0.0;
float apogee_base_ratio = 0.75; 
float apogee_difference_ratio = 0.00;
float const apogee_offset_height = 15.00;

float calibrated_atitude = 0.00;

/*
Units
Pressure - mbars - double
Temperature - Celcius - double
*/

// https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
// The altitude equation is for absolute altitude.
// calibraing : 1 = True, 0 = False
float const alt_offset_height = 20.00;
float const accel_tolerance = 0.1;

float calibrated_altitude = 0.00;

float calculateAltitude(double pressure) {
	double pressure_mb = 33.8639 * (0.2953 * pressure);
	float h_meter = 0.3048 * (1 - pow((pressure_mb / 1013.25), 0.190284)) * 145366.54;
	if (is_calibrated)
	{
      // Clear altitude history
      memset(altitude_history, 0, 3);
	}
	else
	{
      // Relative Altitude
      altitude_history[2] = altitude_history[1];
      altitude_history[1] = altitude_history[0];
      altitude_history[0] = h_meter - global_mission_data.ALTITUDE_OFFSET;
	}
  return h_meter - global_mission_data.ALTITUDE_OFFSET;
}

float calculate_abs_altitude(double pressure) {
	double pressure_mb = 33.8639 * (0.2953 * pressure);
	float h_meter = 0.3048 * (1 - pow((pressure_mb / 1013.25), 0.190284)) * 145366.54;
	return h_meter;
}

// Idea is to calculateAltitude then immediately call this function
// to detemrine state.
void determineState(double altitude){
    // LAUNCH_PAD state
    if (strcmp(global_mission_data.STATE, "LAUNCH_PAD") == 0) {
        if (altitude > lower_altitude_threshold) {
            char _state[] = "ASCENT";
            memcpy(global_mission_data.STATE, _state, sizeof(_state));
        }
    }
    else if (strcmp(global_mission_data.STATE, "ASCENT") == 0) {
        if (altitude > max_altitude) max_altitude = altitude;
        apogee_difference_ratio = 0;
        // apogee_difference_ratio = apogee_offset_height / max_altitude;

        if (fmax(altitude_history[0], altitude_history[1]) < max_altitude - 15) {
            mec_wire_enable = 1;

            char _state[] = "APOGEE";
            memcpy(global_mission_data.STATE, _state, sizeof(_state));
        }
    }
    else if (strcmp(global_mission_data.STATE, "APOGEE") == 0) {
        if (altitude > max_altitude * (apogee_base_ratio)) {
            char _state[] = "DESCENT";
            memcpy(global_mission_data.STATE, _state, sizeof(_state));
        }
        else if (altitude < max_altitude * (apogee_base_ratio)) {
            char _state[] = "PROBE_RELEASE";
            memcpy(global_mission_data.STATE, _state, sizeof(_state));
        }
    }
    else if (strcmp(global_mission_data.STATE, "DESCENT") == 0) {
        if (altitude < max_altitude * (apogee_base_ratio)) {
            char _state[] = "PROBE_RELEASE";
            memcpy(global_mission_data.STATE, _state, sizeof(_state));
        }
    }
    else if (strcmp(global_mission_data.STATE, "PROBE_RELEASE") == 0) {
        if (altitude < lower_altitude_threshold) {
            char _state[] = "LANDED";
            memcpy(global_mission_data.STATE, _state, sizeof(_state));
        }
    }
}
