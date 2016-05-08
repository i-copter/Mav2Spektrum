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

#include "include/mavlink_types.h"
#include "include/common/mavlink.h"



mavlink_message_t msg;
mavlink_status_t status;

static int packet_drops = 0;
static int parse_error = 0;


void receive_mavlink(){

if (Serial1.available()){
		if (mavlink_parse_char(MAVLINK_COMM_0,Serial1.read(),&msg,&status))
		{
			digitalWrite(13,HIGH);
			//Serial.printf("Received message with ID %d, sequence: %d from component %d of system %d \n", msg.msgid, msg.seq, msg.compid, msg.sysid);
			switch (msg.msgid){

			case MAVLINK_MSG_ID_GPS_RAW_INT:
				gps_sats = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
				gps_time = (mavlink_msg_gps_raw_int_get_time_usec(&msg)) / 1e6;  // This is actually the time from boot... fix required in mavlink and APM
				gps_hdop = mavlink_msg_gps_raw_int_get_eph(&msg);
				gps_course = mavlink_msg_gps_raw_int_get_cog(&msg);
				gps_speed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
				gps_alt = mavlink_msg_gps_raw_int_get_alt(&msg);
				gps_lon = mavlink_msg_gps_raw_int_get_lon(&msg);
				gps_lat = mavlink_msg_gps_raw_int_get_lat(&msg);
				(mavlink_msg_gps_raw_int_get_fix_type(&msg) >= 3) ? gps_fix=true : gps_fix=false;
				breakTime(gps_time, tmGpsTime);

				break;

			case MAVLINK_MSG_ID_VFR_HUD:
				airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg) * 3.6;  // convert m/s (mavlink) to km/h (spektrum)
				altitude = mavlink_msg_vfr_hud_get_alt(&msg);
				break;

			case MAVLINK_MSG_ID_ATTITUDE:
				roll =  mavlink_msg_attitude_get_roll(&msg) * 57.2958F;
				pitch =  mavlink_msg_attitude_get_pitch(&msg) * 57.2958F;
				yaw =  mavlink_msg_attitude_get_yaw(&msg) * 57.2958F;
				break;

			case MAVLINK_MSG_ID_SYS_STATUS:
				battVoltage = mavlink_msg_sys_status_get_voltage_battery(&msg) / 10;
				battCurrentConsumed = mavlink_msg_sys_status_get_battery_remaining(&msg);
				battCurrent = mavlink_msg_sys_status_get_current_battery(&msg) * 10;
				break;

			default:
				break;
			}
			packet_drops += status.packet_rx_drop_count;
			parse_error += status.parse_error;
		}
	}
digitalWrite(13,LOW);
}
