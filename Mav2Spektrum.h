#ifndef MAV2SPEKTRUM_H_
#define MAV2SPEKTRUM_H_

#include <TimeLib.h>

#define TELEMETRY_SPEED 115200


uint16_t battVoltage = 0;
int16_t battCurrent = 0;
uint8_t battCurrentConsumed = 0; // in %

float gps_alt = 0;
float gps_speed = 0;
float gps_ground = 0;

float airspeed = 0;
float altitude = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

time_t gps_time = 0;
tmElements_t timeE;

uint8_t gps_sats = 0;

#endif
