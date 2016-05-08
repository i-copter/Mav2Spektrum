/**
 * Mav2Spektrum serial to i2c protocol conversion for mavlink to
 * 	Spektrum TM1000
 *
 *  Copyright 2016 by i-copter.com <icopter.productions@gmail.com>
 *
 *	This file is part of some open source application.
 *
 * 	Some open source application is free software: you can redistribute
 * 	it and/or modify it under the terms of the GNU General Public
 * 	License as published by the Free Software Foundation, either
 * 	version 3 of the License, or (at your option) any later version.
 *
 * 	Some open source application is distributed in the hope that it will
 * 	be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * 	of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 * 	You should have received a copy of the GNU General Public License
 *  If not, see <http://www.gnu.org/licenses/>.
 *
 * 	@license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

#ifndef MAV2SPEKTRUM_H_
#define MAV2SPEKTRUM_H_

#include <TimeLib.h>
#include <Time.h>

#define TELEMETRY_SPEED 115200


uint16_t battVoltage = 0;
int16_t battCurrent = 0;
uint8_t battCurrentConsumed = 0; // in %

float gps_ground = 0;
float altitude = 0;
float airspeed = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

time_t gps_time = 0;
tmElements_t tmGpsTime;

uint8_t gps_sats = 0;
uint16_t gps_hdop = 0;
uint16_t gps_course = 0;
uint16_t gps_speed = 0;
uint32_t gps_alt = 0;
uint32_t gps_lon = 0;
uint32_t gps_lat = 0;

int8_t	deglat;
int8_t	minlat;
int8_t	seclat;
int8_t	subSeclat;
int8_t	deglon;
int8_t	minlon;
int8_t	seclon;
int8_t	subSeclon;
int8_t	flaggps;

bool gps_fix = false;


#endif
