/* #include "MS5607SPI.h"
#include "ICM42688P/ICM42688PSPI.h"
#include <math.h>
#include <string.h>
#include <global.h>

float const lower_altitude_threshold = 5.0;
float max_altitude = 0.0;

float apogee_base_ratio = 0.75;
float apogee_difference_ratio = 0.00;

float const alt_offset_height = 20.00;
float const accel_tolerance = 0.1;

float calibrated_altitude = 0.00;

float calculateAltitude(double pressure, int calibrating) {
	double pressure_mb = 33.8639 * (0.2953 * pressure);
	float h_meter = 0.3048 * (1 - pow((pressure_mb / 1013.25), 0.190284)) * 145366.54;
	if (calibrating == 1)
	{
		// Absolute Altitude of the ground station
	    calibrated_altitude = h_meter;
	    // Relative Altitude of GCS
	    return 0;
	}
	else
	{
	      // Relative Altitude of CanSat
	   return h_meter - calibrated_altitude;
	}
	//return 100.0;
}

// Rather than using a history of altitudes, we figure out when acceleration matches the acceleration of gravity.
// Includes a tolerance to ensure apogee state is reached.
void determine_state(double altitude, ICM42688P_AccelData data)
{
  if (altitude > lower_altitude_threshold && strcmp(global_mission_data.STATE, "LAUNCH_PAD"))
  {
    char _state[] = "ASCENT";
    memcpy(global_mission_data.STATE, _state, sizeof(_state));
  }
  else if (altitude < lower_altitude_threshold && strcmp(global_mission_data.STATE, "PROBE_RELEASE"))
  {
    char _state[] = "LANDED";
    memcpy(global_mission_data.STATE, _state, sizeof(_state));
  }
  else if (data.accel_z >= (1 - accel_tolerance))
  {
    if (max_altitude == 0)
    {
      max_altitude = altitude;
      apogee_difference_ratio = alt_offset_height / max_altitude;
      char _state[] = "APOGEE";
      memcpy(global_mission_data.STATE, _state, sizeof(_state));
    }
    else if (max_altitude * (apogee_base_ratio + apogee_difference_ratio) > altitude)
    {
      char _state[] = "PROBE_RELEASE";
      memcpy(global_mission_data.STATE, _state, sizeof(_state));
      // DO ACTUATOR STUFF
    }
    else
    {
      char _state[] = "DESCENT";
      memcpy(global_mission_data.STATE, _state, sizeof(_state));
    }
  }
}
*/