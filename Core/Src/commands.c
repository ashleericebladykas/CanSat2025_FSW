/*
 * commands.c
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */


#include "commands.h"

static uint8_t sim_enabled = 0;

//static CMD_STATUS trim_xbee_header(const char* incoming, const char* trimmed){
//
//
//	return CMD_OK;
//}

static void get_current_token(const uint8_t token_size, char* token, char** cmd_ptr){
	uint8_t char_cnt = 0;
	for(uint8_t i = 0; i < token_size; i++){
		token[i] = **cmd_ptr;
		(*cmd_ptr)++;
		if(token[i] == ',' || token[i] == '\0'){
			token[i] = '\0';
			break;
		}else{
			char_cnt++;
		}
	}

	// Assign CMD_ECHO to the current command
	memset(global_mission_data.CMD_ECHO, 0, 10);
	strncat(global_mission_data.CMD_ECHO, token, char_cnt);
}

static uint8_t verify_time_format(const char* time){
	if (time == NULL || strlen(time) != 8)
		return 0;
	
	uint8_t numTokens = 1;
	char* endptr;
	int val;

    // Keep printing tokens while one of the
    // delimiters present in str[].
	char* token = strtok(time, ":");
    while (token != NULL) {
		// Ensure that the time integer is valid
		val = strtol(token, &endptr, 10);
		if (endptr == token || *endptr != '\0')
			return 0;

		switch (numTokens) {
			case 1:
				if (val > 23 || val < 0)
					return 0;
				break;
			case 2:
				if (val > 60 || val < 0)
					return 0;
				break;
			case 3:
				if (val > 60 || val < 0)
					return 0;
				break;
			default:
				break;
		}
		token = strtok(NULL, ":");
		numTokens++;
    }

	return numTokens == 3;
}

static CMD_STATUS verify_cmd_header(const char* incoming){
	if(strncmp(incoming, "CMD,3174,", CMD_HEADER_SIZE) != 0){
		return CMD_HEADER_INVLD;
	}
	return CMD_OK;
}

static CMD_STATUS perform_CX(const char* incoming, char* cmd_ptr){
	char on_off[MAX_CX_SIZE];
	get_current_token(MAX_CX_SIZE, on_off, &cmd_ptr);

	if(!strncmp(on_off, "ON", 2)){
	    // Set global flag
		telemetry_enable = 1;
		return CMD_CX_ON;
	}else if(!strncmp(on_off, "OFF", 3)){
		telemetry_enable = 0;
		return CMD_CX_OFF;
	}else{
	    //invalid, do nothing, or set error code somehow?
		return CMD_CX_INVLD;
	}
}

static CMD_STATUS perform_ST(const char* incoming, char* cmd_ptr){
	char time[MAX_ST_SIZE];
	get_current_token(MAX_ST_SIZE, time, &cmd_ptr);

	if(!strncmp(time, "GPS", 3)){
		gps_time_enable = 1;
		return CMD_ST_GPS;
	}else{
	  //first verify if in right format (not null pointer, colons in right place, all digits)
	  //then use macro, or helper func another file kinda needs to be made for that (including verifying above)
	  if(verify_time_format(time)){
		  gps_time_enable = 0;
		  strncpy(global_mission_data.MISSION_TIME, cmd_ptr + 3, 8);
		  return CMD_ST_UTC;
	  }else{
		  return CMD_ST_INVLD;
	  }
	}
}

static CMD_STATUS perform_SIM(const char* incoming, char* cmd_ptr){
	char sim[MAX_SIM_SIZE];
	get_current_token(MAX_SIM_SIZE, sim, &cmd_ptr);

	if(!strncmp(sim, "ENABLE", 6)){
		sim_enabled = 1;
		return CMD_SIM_E;
	}else if(!strncmp(sim, "ACTIVATE", 8)){
		if(sim_enabled){
			global_mission_data.MODE = 'S';
			return CMD_SIM_S;
		}
		return CMD_SIM_NE;
	}else if(!strncmp(sim, "DISABLE", 7)){
		sim_enabled = 0;
		global_mission_data.MODE = 'F';
		return CMD_SIM_F;
	}else{
		return CMD_SIM_INVLD;
	}
}

static CMD_STATUS perform_SIMP(const char* incoming, char* cmd_ptr){
	char simp[MAX_SIMP_SIZE];
	get_current_token(MAX_SIMP_SIZE, simp, &cmd_ptr);

	// Verify that the pressure parameter is an integer
	char* endptr;
	int pressure = strtol(simp, &endptr, 10);
	if (endptr == simp || *endptr != '\0')
		return CMD_SIMP_INVLD;

	// Convert from Pa to kPa
	simulated_pressure = (double)pressure / 1000;
	return CMD_SIMP_RX;
}

//static CMD_STATUS perform_CAL(){
//	if (!strncmp(global_mission_data.STATE, "LAUNCH_PAD", 10))
//		return CMD_CAL_INVLD;
//
//	global_mission_data.ALTITUDE_OFFSET = global_mission_data.ALTITUDE;
//	is_calibrated = 1;
//	return CMD_CAL_RX;
//}

static CMD_STATUS perform_MEC(const char* incoming, char* cmd_ptr){
	char mec[MAX_MEC_SIZE];
	get_current_token(MAX_MEC_SIZE, mec, &cmd_ptr);

	if(!strncmp(mec, "WIRE", 4)){
		if (!strncmp(mec + 5, "ON", 2)) {
			mec_wire_enable = 1;
			return CMD_MEC_WIRE;
		} else if (!strncmp(mec + 5, "OFF", 3)) {
			mec_wire_enable = 0;
			return CMD_MEC_WIRE;
		}
	}else{
		return CMD_MEC_INVLD; //add the rest of status codes as more mechanisms added
	}
}

CMD_STATUS process_command(char* incoming){
	CMD_STATUS cmd_status;
	//if in debug mode, ignore xbee header trimming. or just comment out i guess
//		char trimmed_cmd_str[32];
//		memset(trimmed_cmd_str, 0);
//		cmd_status = trim_xbee_header(incoming, trimmed_cmd_str);
//
//		if(cmd_status != CMD_OK){
//			return cmd_status;
//		}

	//make sure the beginning is CMD,3174,
	cmd_status = verify_cmd_header(incoming);

	if(cmd_status != CMD_OK){
		//uart print error/status?
		return cmd_status;
	}

	char cmd[MAX_CMD_SIZE];
	char* cmd_ptr = incoming + CMD_HEADER_SIZE; //magic numbers, define later todo

	get_current_token(MAX_CMD_SIZE, cmd, &cmd_ptr);

	if(!strncmp(cmd, "CX", 2)){ //Payload Telemetry On/Off Command
		cmd_status = perform_CX(incoming, cmd_ptr);
	}else if(!strncmp(cmd, "ST", 2)){ //Set Time
		cmd_status = perform_ST(incoming, cmd_ptr);
	}else if(!strncmp(cmd, "SIM", 3)){ //Simulation Mode Control Command
		cmd_status = perform_SIM(incoming, cmd_ptr);
	}else if(!strncmp(cmd, "SIMP", 4)){ //Simulated Pressure Data (to be used in Simulation Mode only)
		cmd_status = perform_SIMP(incoming, cmd_ptr);
	}else if(!strncmp(cmd, "CAL", 3)){ //Calibrate Altitude to Zero
		cmd_status = perform_CAL();
	}else if(!strncmp(cmd, "MEC", 3)){ //Mechanism actuation command
		cmd_status = perform_MEC(incoming, cmd_ptr);
	}else{
		return CMD_INVLD;
	}

	return cmd_status;
}

//TEST METHODS BELOW

static void test_cmd_header(){
	CMD_STATUS cmd_status;
	char test[64];

	//Correct header
	strcpy(test, "CMD,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect beginning of string, 1 char
	strcpy(test, "C,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect beginning of string, 2 chars
	strcpy(test, "CM,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect beginning of string, 3 chars
	strcpy(test, "CDM,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect, No delimiter
	strcpy(test, "CDM3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect, No team ID
	strcpy(test, "CMD,,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect, no rest of message, only header
	strcpy(test, "CMD,3174");
	cmd_status = verify_cmd_header(test);

	cmd_status = cmd_status; //get rid of annoying not used warning
}

void run_command_test_cases(){
	test_cmd_header();
	//add more as needed, test each command
}





