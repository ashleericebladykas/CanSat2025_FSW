/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
// #include "commands.h"
#include "uart_interrupt.h"

#include "../../Drivers/ICM42688P/ICM42688PSPI.h"
#include "../../Drivers/MS5607/MS5607SPI.h"
#include "../../Drivers/BMM150/BMM150SPI.h"
#include "../../Drivers/LC76G/LC76G.h"
#include "../../Drivers/AMT10E2/AMT10E2.h"
#include "../../Drivers/BQ28Z610/BQ28Z610I2C.h"
#include "../../Drivers/DRV8838/DRV8838.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int _write(int fd, char *ptr, int len)
{
  // ignore fd, just send to UART3
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CORDIC_HandleTypeDef hcordic;

FMAC_HandleTypeDef hfmac;

I2C_HandleTypeDef hi2c3;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_IRTIM_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_UART5_Init(void);
static void MX_RNG_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FMAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_IRTIM_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  // MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_UART5_Init();
  MX_RNG_Init();
  MX_USB_Device_Init();
  MX_USART3_UART_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */
  // Feedback LED
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);

  // Enable GPS and XBEE
  HAL_GPIO_WritePin(XBEE_RST_GPIO_Port, XBEE_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(3000); // wait for the Xbee to get brought back up again

  // Disable ALL chip selects
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMP_nCS_GPIO_Port, BMP_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAG_nCS_GPIO_Port, MAG_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAGEXT_nCS_GPIO_Port, MAGEXT_nCS_Pin, GPIO_PIN_SET);

  // Initialize ICM42688P (IMU)
  ICM42688P_init(&hspi2, IMU_nCS_GPIO_Port, IMU_nCS_Pin);

  // Initialize pressure sensor (MS5607)
  MS5607_Init(&hspi2, BMP_nCS_GPIO_Port, BMP_nCS_Pin);

  // Initialize magnetometer (BMM150)
  struct bmm150_dev bmm150 = BMM150_spi_init(&hspi2, MAG_nCS_GPIO_Port, MAG_nCS_Pin);

  // Initialize GPS (LC76G)
  LC76G_init();

  // Initialize quadrature encoder (AMT10E2)
  QENC_Init_Encoder0();

  // drv8838_init(&htim15, DRV_DIR_GPIO_Port, DRV_DIR_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // DELETE:
  int i = 0;
  // Generic temporary variable for use in sprintf() calls, etc.
  int str_len = 0;

  MS5607Readings bmp_data;      // pressure data
  ICM42688P_AccelData imu_data; // accelerometer / gyro data
  BMM150_mag_data mag_data;     // DELETE: magnetometer data (unused)
  LC76G_gps_data gps_data;      // GPS data

  uint8_t update_time = 0;

  // How long (in seconds) should the resistor stay actuated until safety shutoff?
  uint8_t super_hot_resistor_cycle_limit = 45;
  // How long (in seconds) has the resistor been actuated for?
  uint8_t super_hot_resistor_cycles = 0;

  // DELETE:
  char command[64] = {0};

  // Holds the number of quadrature encoder revolutions measured in the
  // previous and current telemetry packet, respectively (used to calculate
  // auto-gyro rotation rate)
  int16_t prev_enc_count = 0;
  int16_t enc_count = 0;

  // Initialize the global mission_data struct
  init_mission_data();

  while (1)
  {
    // enable interrupts
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    // receive command (22 bytes max)
    uint8_t rx_buff[22];

    HAL_UART_Receive_IT(&huart3, rx_buff, 22);
    // HAL_UART_Transmit(&huart3, rx_buff, sizeof(rx_buff), HAL_MAX_DELAY);

    // step1: convert rx_buff array of "uint8_t"s into array of "chars"
    char *char_array = (char *)rx_buff;
    char rx_string[22];

    // step2: convert array of chars into string  (https://www.geeksforgeeks.org/convert-character-array-to-string-in-c/)
    strncpy(rx_string, char_array, 22);
    // strcpy(global_mission_data.CMD_ECHO, rx_string);

    // Add null terminator to end of command string
    rx_string[22] = '\0';
    // HAL_UART_Transmit(&huart3, rx_string, sizeof(rx_string), HAL_MAX_DELAY);

    // step3: use string::compare and chop off the first 12 characters of the string (https://cplusplus.com/reference/string/string/compare/)
    // char cmd_prefix[12] = "CMD,3174,CX,";
    // cmd_length = str_len(cmd_prefix);
    // const char *sub_cmd = &rx_string[cmd_length];
    // if (strncmp(rx_string, cmd_prefix, cmd_length) != 0) {
    //
    // if (strncmp(&sub_cmd, "SIM", cmd_length) != 0) {
    // break;
    //}
    // elif (strncmp(&sub_cmd, "CAL", cmd_length) != 0) {
    // break;
    //}
    // elif (strncmp(&sub_cmd, "MEC", cmd_length) != 0) {
    // break;
    //}

    // CX ON command -> start transmitting telemetry packets
    if (strncmp(rx_string, "CMD,3174,CX,ON", 14) == 0)
    {
      //    	GPIO_TypeDef* CXON_Port = "A";
      //    	uint16_t CXON_Pin = 8;
      //		HAL_GPIO_WritePin(CXON_Port,CXON_Pin, GPIO_PIN_SET);
      //		HAL_GPIO_WritePin(CXON_Port,CXON_Pin, GPIO_PIN_RESET);
      //    	 __SEV();
      // set global telemetry flag
      telemetry_enable = 1;

      // set command echo in the global mission struct
      char c_echo[] = "CXON";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // CX OFF command -> stop transmitting telemetry packets
    else if (strncmp(rx_string, "CMD,3174,CX,OFF", 15) == 0)
    {
      // reset global telemetry flag
      telemetry_enable = 0;

      // set command echo
      char c_echo[] = "CXOFF";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // ST command -> set mission time
    else if (strncmp(rx_string, "CMD,3174,ST,", 12) == 0)
    {
      // parse the timestamp to set to
      char arg[9];
      char *time_str = rx_string + 12;
      strncpy(arg, time_str, 9);

      // if a manual timestamp has been input...
      if (strlen(arg) == 8)
      {
        // set mission time
        char *str_end;
        strncpy(global_mission_data.MISSION_TIME, time_str, 9);
        // Set a flag telling us to update the RTC
        update_time = 1;
        // stop reading time from GPS
        gps_time_enable = 0;
      }
      // read time from GPS
      else if (strncmp(time_str, "GPS", 3))
      {
        gps_time_enable = 1;
      }
      else
      {
        // if the string is not 8 characters long, set it to "00:00:00"
        strcpy(global_mission_data.MISSION_TIME, "00:00:00");
        gps_time_enable = 0;
      }
      // set command echo
      char c_echo[] = "ST";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // SIM ENABLE command -> allow simulation mode to be activated
    else if (strncmp(rx_string, "CMD,3174,SIM,ENABLE", 19) == 0)
    {
      // set global flag
      simulation_pre = 1;
      // set command echo
      char c_echo[] = "SIMENABLE";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // SIM ACTIVATE command -> turn simulation mode on
    else if (strncmp(rx_string, "CMD,3174,SIM,ACTIVATE", 21) == 0)
    {
      // check that simulation mode has been activated
      if (simulation_pre == 1)
      {
        // set global simulation flag
        simulation_enable = 1;
        // make first simulated pressure value match actual value
        simulated_pressure = global_mission_data.PRESSURE;
        // set command echo
        char c_echo[] = "SIMACT";
        strcpy(global_mission_data.CMD_ECHO, c_echo);
      }
    }
    // SIM DISABLE command -> turn simulation mode off
    else if (strncmp(rx_string, "CMD,3174,SIM,DISABLE", 20) == 0)
    {
      // reset global flag
      simulation_enable = 0;
      // set command echo
      char c_echo[] = "SIMDIS";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // SIMP command -> add simulated pressure data
    else if (strncmp(rx_string, "CMD,3174,SIMP,", 14) == 0)
    {
      // parse inputted pressure data
      char *pressure_str = rx_string + 14;
      char *str_end;
      long pressure_pa = atof(rx_string + 14);
      // if (str_end == pressure_str || *str_end != '\0')
      // it wasn't a valid number
      // set simulated pressure to parsed value
      simulated_pressure = pressure_pa;

      // set command echo
      char c_echo[] = "SIMP";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // CAL command -> calibrate altitude
    else if (strncmp(rx_string, "CMD,3174,CAL", 12) == 0)
    {
      // set global flag
      is_calibrated = 1;
      // set command echo
      char c_echo[] = "CAL";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // MEC WIRE ON command -> actuate resistor-filament system
    else if (strncmp(rx_string, "CMD,3174,MEC,WIRE,ON", 20) == 0)
    {
      // set global falg
      mec_wire_enable = 1;
      // set command echo
      char c_echo[] = "MECON";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
      // DELETE:
      HAL_UART_Transmit(&huart3, c_echo, sizeof(c_echo), HAL_MAX_DELAY);
    }
    // MEC WIRE OFF command -> manually stop actuating resistor-filament system
    else if (strncmp(rx_string, "CMD,3174,MEC,WIRE,OFF", 21) == 0)
    {
      // disable global flag
      mec_wire_enable = 0;
      // set command echo
      char c_echo[] = "MECOFF";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // DELETE:
    else
    {
      //      HAL_UART_Transmit(&huart3, rx_string, sizeof(rx_string), HAL_MAX_DELAY);
    }
    // clear command buffer
    memset(rx_buff, 0, sizeof(rx_buff));

    // Receive command from ground station
    // HAL_UART_Receive(&huart3, command, 64, 10);
    // process_command(command);

    // read pressure sensor values
    bmp_data = MS5607ReadValues();
    // read IMU values
    imu_data = ICM42688P_read_data();
    // DELETE: broken/unused sensor data
    // mag_data = BMM150_read_mag_data(&bmm150);
    // gps_data = LC76G_read_data();
    // read quadrature encoder values
    enc_count = QENC_Get_Encoder0_Count();

    // record sensor data into the global mission data struct
    global_mission_data.TEMPERATURE = bmp_data.temperature_C;

    // in simulation mode, update pressure to match what is parsed from command
    if (simulation_enable == 1)
    {
      global_mission_data.PRESSURE = simulated_pressure;
      global_mission_data.MODE = 'S'; // set mode to 'S' for simulation
    }
    // otherwise, update pressure to match data read from sensor
    else
    {
      global_mission_data.PRESSURE = bmp_data.pressure_kPa;
      global_mission_data.MODE = 'F'; // set mode to 'F' for flight
    }
    // if the calibrating flag is true, calibrate the altitude
    global_mission_data.ALTITUDE = calculateAltitude(global_mission_data.PRESSURE);
    if (is_calibrated == 1)
    {
      global_mission_data.ALTITUDE_OFFSET = global_mission_data.ALTITUDE; // set the offset to the current altitude
      is_calibrated = 0;                                                  // reset the flag
    }
    // Use altitude to update the mission state as needed
    determineState(global_mission_data.ALTITUDE);

    // update battery voltage
    uint16_t battery_mV = 0;                                    // temp variable to hold the battery voltage in millivolts
    BQ28Z610_ReadVoltage(&hi2c3, &battery_mV);                  // read battery voltage from BQZ
    global_mission_data.VOLTAGE = (float)(battery_mV) / 1000.0; // convert from mV to V

    // read gyro data
    global_mission_data.GYRO_R = imu_data.gyro_r;
    global_mission_data.GYRO_P = imu_data.gyro_p;
    global_mission_data.GYRO_Y = imu_data.gyro_y;

    // calculates the auto gyro rotation rate in degrees per second according to:
    // (current count) - (previous count) * (360 degrees / 1 revolution) * (1 revolution / 120 counts)
    global_mission_data.AUTO_GYRO_ROTATION_RATE = (enc_count - prev_enc_count) * 3;

    // read gyro acceleration data
    global_mission_data.ACCEL_R = imu_data.accel_r;
    global_mission_data.ACCEL_P = imu_data.accel_p;
    global_mission_data.ACCEL_Y = imu_data.accel_y;

    // update magnetometer; dummy data
    global_mission_data.MAG_R = rand() % 1000 / 1000.0; // mag_r
    global_mission_data.MAG_P = rand() % 1000 / 1000.0; // mag_p
    global_mission_data.MAG_Y = rand() % 1000 / 1000.0; // mag_y
    /*
    global_mission_data.MAG_R =  mag_data.x; // mag_r
    global_mission_data.MAG_P = mag_data.y; // mag_p
    global_mission_data.MAG_Y = mag_data.z; // mag_y
    */

    // update GPS data; dummy data
    /*str_len = sprintf(global_mission_data.GPS_TIME, "%d:%d:%d",
                     gps_data.time_H,
                     gps_data.time_M,
                     gps_data.time_S);
    global_mission_data.GPS_ALTITUDE = gps_data.altitude;
    global_mission_data.GPS_LATITUDE = gps_data.lat;
    global_mission_data.GPS_LONGITUDE = gps_data.lon;
    global_mission_data.GPS_SATS = gps_data.num_sat_used;*/
    strcpy(global_mission_data.GPS_TIME, "XX:XX:XX");
    global_mission_data.GPS_ALTITUDE = calculate_abs_altitude(global_mission_data.PRESSURE) + ((rand() % 200 / 10) - 10);
    global_mission_data.GPS_LATITUDE = 38.3879 + ((rand() % 20 - 10) / 1000);
    global_mission_data.GPS_LONGITUDE = 79.5836 + ((rand() % 30 - 15) / 1000);
    global_mission_data.GPS_SATS = rand() % 8 + 4;

    // send the packet if telemetry is enabled
    if (telemetry_enable)
    {
      // create an empty buffer for the telemetry packet string
      char telemetry_string[200];
      // fill the buffer with the first half of the packet
      str_len = sprintf(telemetry_string, "%d,%s,%ld,%c,%s,%3.1f,%.1f,%.1f,%.1f,%d,%d,%d",
                        global_mission_data.TEAM_ID,      // team id (3174)
                        global_mission_data.MISSION_TIME, // mission time
                        global_mission_data.PACKET_COUNT, // packet count
                        global_mission_data.MODE,         // mode
                        global_mission_data.STATE,        // state
                        global_mission_data.ALTITUDE,     // calibrated altitude (m)
                        global_mission_data.TEMPERATURE,  // temperature (C)
                        global_mission_data.PRESSURE,     // pressure (kPa)
                        global_mission_data.VOLTAGE,      // battery voltage (V)
                        global_mission_data.GYRO_R,       // gyro roll (degrees/s)
                        global_mission_data.GYRO_P,       // gyro pitch (degrees/s)
                        global_mission_data.GYRO_Y        // gyro yaw (degrees/s)
      );
      // str_len = sizeof(telemetry_string);
      // send the first part of the packet over UART
      HAL_UART_Transmit(&huart3, telemetry_string, str_len, HAL_MAX_DELAY);
      // clear the buffer
      memset(telemetry_string, 0, sizeof(telemetry_string));
      // fill the buffer with the second half of the packet
      str_len = sprintf(telemetry_string, ",%d,%d,%d,%.1f,%.1f,%.1f,%d,%s,%.1f,%.4f,%.4f,%d,%s",
                        global_mission_data.ACCEL_R,                 // accelerometer roll (degrees/s^2)
                        global_mission_data.ACCEL_P,                 // accelerometer pitch (degrees/s^2)
                        global_mission_data.ACCEL_Y,                 // accelerometer yaw (degrees/s^2)
                        global_mission_data.MAG_R,                   // magnetometer roll
                        global_mission_data.MAG_P,                   // magnetometer pitch
                        global_mission_data.MAG_Y,                   // magnetometer yaw
                        global_mission_data.AUTO_GYRO_ROTATION_RATE, // auto-gyro rotation rate (rps)
                        global_mission_data.GPS_TIME,                // GPS time
                        global_mission_data.GPS_ALTITUDE,            // GPS (absolute) altitude (m)
                        global_mission_data.GPS_LATITUDE,            // GPS latitude
                        global_mission_data.GPS_LONGITUDE,           // GPS longitude
                        global_mission_data.GPS_SATS,                // # of connected GPS satellites
                        global_mission_data.CMD_ECHO                 // tracks previously received command
      );
      // send the second half of the packet over UART
      HAL_UART_Transmit(&huart3, telemetry_string, str_len, HAL_MAX_DELAY);

      /*char test_string[30];
      str_len = sprintf(test_string, "accel_z: %d", imu_data.accel_z);
      HAL_UART_Transmit(&huart3, test_string, str_len, HAL_MAX_DELAY);*/

      // increment packet count once the entire packet has been transmitted
      global_mission_data.PACKET_COUNT = global_mission_data.PACKET_COUNT + 1;
    }

    // Actuate the resistor
    if (mec_wire_enable == 1)
    {
      // set the resistor pin high
      HAL_GPIO_WritePin(DRV_PWM_GPIO_Port, DRV_PWM_Pin, GPIO_PIN_SET);
      // increment the number of seconds the resistor has been active for
      super_hot_resistor_cycles += 1;

      // if the duration of the resistor's actuation exceeds the safety limit, turn it off
      if (super_hot_resistor_cycles >= super_hot_resistor_cycle_limit)
      {
        HAL_GPIO_WritePin(DRV_PWM_GPIO_Port, DRV_PWM_Pin, GPIO_PIN_RESET);
        super_hot_resistor_cycles = 0;
        mec_wire_enable = 0;
      }
    }
    // Manually turn off the resistor
    else
    {
      // turn resistor off
      HAL_GPIO_WritePin(DRV_PWM_GPIO_Port, DRV_PWM_Pin, GPIO_PIN_RESET);
      super_hot_resistor_cycles = 0;
    }

    // After math has been calculated, set the previous encoder count for the
    // next cycle
    prev_enc_count = enc_count;

    // Update mission time
    if (update_time)
    {
      // Initialize RTC to 00:00:00
      RTC_TimeTypeDef sTime = {0};
      RTC_DateTypeDef sDate = {0};

      //
      char *string_remainder;
      sTime.Hours = strtol(global_mission_data.MISSION_TIME, &string_remainder, 10);
      sTime.Minutes = strtol(++string_remainder, &string_remainder, 10);
      sTime.Seconds = strtol(++string_remainder, &string_remainder, 10);

      if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
      {
        Error_Handler();
      }

      sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
      sDate.Month = RTC_MONTH_JANUARY;
      sDate.Date = 2;
      sDate.Year = 70;

      if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
      {
        Error_Handler();
      }

      update_time = 0;
    }

    if (!gps_time_enable)
    {
      // Update mission time
      RTC_DateTypeDef getDate = {0};
      RTC_TimeTypeDef getTime = {0};

      // check for valid time
      if (HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN) != HAL_OK)
      {
        Error_Handler();
      }

      // Required call to GetDate to unlock time registers
      if (HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN) != HAL_OK)
      {
        Error_Handler();
      }

      // write time from GPS to the mission time struct
      snprintf(global_mission_data.MISSION_TIME, 9, "%02d:%02d:%02d",
               getTime.Hours, getTime.Minutes, getTime.Seconds);
    }

    // Delay for 1000ms to achieve the required 1Hz transmission rate
    // NOTE: adjust to 900? or are the two delays independent?
    HAL_Delay(1000);

    // heartbeat LED
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);

  /** Enable the SYSCFG APB clock
   */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
   */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_LSE;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 32768);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CORDIC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */
}

/**
 * @brief FMAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10B17DB5;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */
}

/**
 * @brief IRTIM Initialization Function
 * @param None
 * @retval None
 */
static void MX_IRTIM_Init(void)
{

  /* USER CODE BEGIN IRTIM_Init 0 */

  /* USER CODE END IRTIM_Init 0 */

  /* USER CODE BEGIN IRTIM_Init 1 */

  /* USER CODE END IRTIM_Init 1 */
  /* USER CODE BEGIN IRTIM_Init 2 */

  /* USER CODE END IRTIM_Init 2 */
}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the reference Clock input
   */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
  sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV1;
  sEncoderIndexConfig.Filter = 0;
  sEncoderIndexConfig.FirstIndexEnable = DISABLE;
  sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
  sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
  if (HAL_TIMEx_ConfigEncoderIndex(&htim1, &sEncoderIndexConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
  sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV1;
  sEncoderIndexConfig.Filter = 0;
  sEncoderIndexConfig.FirstIndexEnable = DISABLE;
  sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
  sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
  if (HAL_TIMEx_ConfigEncoderIndex(&htim8, &sEncoderIndexConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

// Needed to facilitate DMA transfer from GPS module
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART5)
  {
    // gps_dma_buffer now contains 'Size' bytes of received data

    // Parse data stored in DMA buffer
    LC76G_parse_data();

    // Empty the DMA buffer
    memset(gps_dma_buffer, 0, GPS_DMA_BUFFER_SIZE);

    // Restart DMA reception for the next burst
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, gps_dma_buffer, GPS_DMA_BUFFER_SIZE);
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STAT_BKUP_Pin | EN_5V_Pin | CAM1_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRV_PWM_Pin | DRV_DIR_Pin | CAM0_CTRL_Pin | XBEE_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU_nCS_Pin | MAGEXT_nCS_Pin | MAG_nCS_Pin | BMP_nCS_Pin | GPS_RST_Pin | USR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STAT_BKUP_Pin EN_5V_Pin CAM1_CTRL_Pin */
  GPIO_InitStruct.Pin = STAT_BKUP_Pin | EN_5V_Pin | CAM1_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CHG_STAT2_Pin CHG_STAT1_Pin */
  GPIO_InitStruct.Pin = CHG_STAT2_Pin | CHG_STAT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_32K_Pin */
  GPIO_InitStruct.Pin = CLK_32K_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLK_32K_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_DIR_Pin CAM0_CTRL_Pin XBEE_RST_Pin */
  GPIO_InitStruct.Pin = DRV_DIR_Pin | DRV_PWM_Pin | CAM0_CTRL_Pin | XBEE_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_nCS_Pin MAGEXT_nCS_Pin MAG_nCS_Pin BMP_nCS_Pin
                           USR_LED_Pin */
  GPIO_InitStruct.Pin = IMU_nCS_Pin | MAGEXT_nCS_Pin | MAG_nCS_Pin | BMP_nCS_Pin | USR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_IR;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB9);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void SendTelemetry(void *argument)
{
  /* USER CODE BEGIN SendTelemetry */
// this can probably be dynamically typed to take the sizeof() each property instead of being hardcoded
// unsigned int property_sizes[] = {2, 4, 4, 1, STATE_TEXT_LEN, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 1, CMD_ECHO_LEN};

/* Infinite loop */
/*
for (;;)
{
  // create buffer to copy mission data into
  // +19 for all necessary commas
  char telemetry_string[sizeof(global_mission_data) + 19];
  char *data_pointer = &global_mission_data;

  // external index!!
  unsigned int s_index = 0;

  // find length of array
  unsigned int num_properties = sizeof(property_sizes) / sizeof(unsigned int);
  for (unsigned int k = 0; k < num_properties; k++)
  {
    // memcpy() transfers 'k' bytes of data from the struct to the string
    memcpy(*telemetry_string + index, data_pointer, property_sizes[k]);

    // move the data pointer and the string pointer separately (commas in the
    // resulting string mean the two positions are not always equal)
    data_pointer += property_sizes[k];
    s_index += property_sizes[k];

    // don't add a comma after the last property!
    if (k < num_properties - 1)
    {
      telemetry_string[s_index] = ',';
      s_index = s_index + 1;
    }
  }

  // after copying all the data over, add a null terminator at the end
  telemetry_string[s_index] = '\0';

  // char test_string[] = "TESTLEMETRY";

  // HAL_UART_Transmit(&huart4, test_string, sizeof(test_string), 250);

  /*
  telemetry_string[] now contains global_mission_data formatted as a string of characters
  data in LITTLE ENDIAN format:
    TEAM_ID = 3174 = 0x0C66 and is stored as ASCII codes 0x66 ('f') followed by 0x0C (NP form feed) in the string

  string format is as follows in terms of bytes:
    string[0:1] = TEAM_ID[1:0]
    string[2:5] = MISSION_TIME[3:0] <-- format to UTC time (hh:mm:ss)
    string[6:9] = PACKET_COUNT[3:0]
    string[10] = MODE
    string[11:20] = STATE[0:9]
    string[21:24] = ALTITUDE[3:0]
    string[25:28] = TEMPERATURE[3:0]
    string[29:32] = PRESSURE[3:0]
    string[33:36] = VOLTAGE[3:0]
    string[37:40] = GYRO_R[3:0]
    string[41:44] = GYRO_P[3:0]
    string[45:48] = GYRO_Y[3:0]
    string[49:52] = ACCEL_R[3:0]
    string[53:56] = ACCEL_P[3:0]
    string[57:60] = ACCEL_Y[3:0]
    string[61:64] = MAG_R[3:0]
    string[65:68] = MAG_P[3:0]
    string[69:72] = MAG_Y[3:0]
    string[73:74] = AUTO_GYRO_ROTATION_RATE[1:0]
    string[75:78] = GPS_TIME[3:0]
    string[79:82] = GPS_ALTITUDE[3:0]
    string[83:86] = GPS_LATITUDE[3:0]
    string[87:90] = GPS_LONGITUDE[3:0]
    string[91] = GPS_SATS
    string[92:101] = CMD_ECHO[0:9]
  */

// HAL_Delay(1000);
//}
//}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
