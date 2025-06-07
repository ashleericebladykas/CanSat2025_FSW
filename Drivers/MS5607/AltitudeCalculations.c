/*#include "MS5607SPI.h"
#include <math.h>
#include <global.h>

float const lower_altitude_threshold = 5.0;

// Index is by seconds ago the value was calculated
float altitude_history[] = {0, 0, 0};

float max_altitude = 0.0;
float apogee_base_ratio = 0.75; 
float apogee_difference_ratio = 0.00;
float const apogee_offset_height = 20.00;

float calibrated_atitude = 0.00;

// Units
// Pressure - mbars - double
// Temperature - Celcius - double


// https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
// The altitude equation is for absolute altitude.
// calibraing : 1 = True, 0 = False
float caluclateAltitude(double pressure, int calibrating){
    float pressure_mb = 33.8639 * (0.2953 * pressure);
	float h_meter = 0.3048 * (1 - pow((pressure_mb / 1013.25), 0.190284)) * 145366.54;er = 0.3048*((1 - pow((pressure/1013.25), 0.190284))*145366.54);
    if (calibrating == 1){
        // Absolute Altitude of the ground station
        calibrated_atitude = h_meter;
        // Relative Altitude of GCS
        return 0;
    }
    else{
        // Relative Altitude
        altitude_history[2] = altitude_history[1];
        altitude_history[1] = altitude_history[0];
        altitude_history[0] = h_meter - calibrated_atitude;
        return altitude_history[0];
    }
}

// Idea is to calculateAltitude then immediately call this function
// to detemrine state.
void determineState(double altitude){
    // Set to ASCENT.
    if(altitude > lower_altitude_threshold 
        && strcmp(global_mission_data.STATE, "LAUNCH_PAD"))
    {
        char _state[] = "ASCENT";
        memcpy(global_mission_data.STATE, _state, sizeof(_state));
    }
    // Set to APOGEE.
    else if (altitude_history[2] > altitude_history[1] && altitude_history[2] > altitude_history[0] 
        && strcmp(global_mission_data.STATE, "ASCENT"))
    {   
        // Calculate where probe should be released at.
        max_altitude = altitude;
        apogee_difference_ratio = apogee_offset_height / max_altitude;

        // Set flag to start heating up the resistor.
        mec_wire_enable = 1;

        char _state[] = "APOGEE";
        memcpy(global_mission_data.STATE, _state, sizeof(_state));
    }
    // Set to DESCENT
    else if(strcmp(global_mission_data.STATE, "APOGEE") 
        && altitude > max_altitude*(apogee_base_ratio + apogee_difference_ratio))
    {
        char _state[] = "DESCENT";
        memcpy(global_mission_data.STATE, _state, sizeof(_state));
    }
    // Set to PROBE_RELEASE
    else if(strcmp(global_mission_data.STATE, "DESCENT") 
        && altitude < max_altitude*(apogee_base_ratio + apogee_difference_ratio))
    {
        char _state[] = "PROBE_RELEASE";
        memcpy(global_mission_data.STATE, _state, sizeof(_state));
    }
    // Set to LANDED
    else if(strcmp(global_mission_data.STATE, "PROBE_RELEASE")
        && altitude < lower_altitude_threshold)
    {
        char _state[] = "LANDED";
        memcpy(global_mission_data.STATE, _state, sizeof(_state));
    }
}
    */
