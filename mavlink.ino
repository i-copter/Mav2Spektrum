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
			//Serial.printf("Received message with ID %d, sequence: %d from component %d of system %d \n", msg.msgid, msg.seq, msg.compid, msg.sysid);
			switch (msg.msgid){

			case MAVLINK_MSG_ID_GPS_RAW_INT:
				gps_sats = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
				gps_time = mavlink_msg_gps_raw_int_get_time_usec(&msg);
				timet = mavlink_msg_gps_raw_int_get_time_usec(&msg);
				Serial.printf("%u\n",gps_time);
				Serial.println(timet);
				//breakTime(gps_time, timeE);
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
				battVoltage = mavlink_msg_sys_status_get_voltage_battery(&msg);
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
}







//	 while(mav_serial.available() > 0) {
//	    uint8_t c = mav_serial.read();
//	    mavlink_parse_char(MAVCOM,c,&msg,&status);
//	    messageCounter = 0;
//	          mavlink_active = 1;
//
//	          switch(msg.msgid)
//	          {
//	          case MAVLINK_MSG_ID_GPS_STATUS :
//	        	  //uint8_t mavlink_msg_gps_status_get_satellites_visible(const mavlink_message_t* msg)
//	        	  sats = mavlink_msg_gps_status_get_satellites_visible(&msg);
//
//	        	  break;
//
//	          case MAVLINK_MSG_ID_SYS_STATUS :
//	        	  cbat_A = mavlink_msg_sys_status_get_load(&msg);
//
//	        	  break;
//
//	          case MAVLINK_MSG_ID_ALTITUDE :
//	        	  gps_alt = (mavlink_msg_altitude_get_altitude_relative(&msg) / 10.0F);
//
////mavlink_msg_altitude_get_altitude_relative
//	          break;
//
////uint32_t mavlink_msg_system_time_get_time_boot_ms(const mavlink_message_t* msg)
//	          default :
//	        	  break;
//	          }
//	          packet_drops += status.packet_rx_drop_count;
//	          parse_error += status.parse_error;
//	 }



/*#include <mavlink.h>
*
* mavlink_message_t msg;
* int chan = 0;
*
*
* while(serial.bytesAvailable > 0)
* {
*   uint8_t byte = serial.getNextByte();
*   if (mavlink_parse_char(chan, byte, &msg))
*     {
*     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
*     }
* }
*/

