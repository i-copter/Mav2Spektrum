#include "include/mavlink_types.h"
#include "include/common/mavlink.h"



mavlink_message_t msg;
mavlink_status_t status;

static int packet_drops = 0;
static int parse_error = 0;

long timet=0;

void receive_mavlink(){

if (Serial1.available()){
		if (mavlink_parse_char(MAVLINK_COMM_0,Serial1.read(),&msg,&status))
		{
			digitalWrite(13,HIGH);
			//Serial.printf("Received message with ID %d, sequence: %d from component %d of system %d \n", msg.msgid, msg.seq, msg.compid, msg.sysid);
			switch (msg.msgid){

			case MAVLINK_MSG_ID_GPS_RAW_INT:
				gps_sats = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
				gps_time = mavlink_msg_gps_raw_int_get_time_usec(&msg);
				//Serial.printf("%u\n",gps_time);
				//Serial.println(timet);
				breakTime(gps_time, timeE);
				//Serial.printf("HH:%d MM:%d SS:%d\n",timeE.Hour,timeE.Minute,timeE.Second);
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
