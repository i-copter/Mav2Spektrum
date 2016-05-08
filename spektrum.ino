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
 *
 * 	GPS Telemetry Section code based on mav2telem
 *      Menno de Gans (joebar.rc@googlemail.com)
 */

#include "spektrum.h"
#include <i2c_t3.h>

uint8_t SpektrumTelemetryBuffer[16];
uint16_t i=0;

void spektrumInit()
{
	// Slave i2c mode, listen to all addresses
	Wire.begin(I2C_SLAVE, 0x03, SpektrumSlaveI2C_MAX, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
	Wire.onRequest(SpektrumSensorRequest);
}

void SpektrumSensorRequest()
{
		switch (Wire.getRxAddr()){

#ifdef SpektrumSlaveI2C_VOLTAGE
		case SpektrumSlaveI2C_VOLTAGE :
		break;
#endif

#ifdef SpektrumSlaveI2C_TEMPERATURE
		case SpektrumSlaveI2C_TEMPERATURE :
		break;
#endif

#ifdef SpektrumSlaveI2C_IHIGH_CURRENT
		case SpektrumSlaveI2C_IHIGH_CURRENT :
			SpektrumTelemetry_IHigh.current =  swap_int16((battCurrent / 0.196791F) / 1000); // / 0.196791
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_IHigh, sizeof(SpektrumTelemetry_IHigh));
		break;
#endif

#ifdef SpektrumSlaveI2C_RSV_04
		case SpektrumSlaveI2C_RSV_04 :
		break;
#endif

#ifdef  SpektrumSlaveI2C_RSV_05
		case SpektrumSlaveI2C_RSV_05 :
		break;
#endif

#ifdef 	SpektrumSlaveI2C_RSV_06
		case SpektrumSlaveI2C_RSV_06 :
		break;
#endif

#ifdef	SpektrumSlaveI2C_RSV_07
		case SpektrumSlaveI2C_RSV_07 :
		break;
#endif

#ifdef	SpektrumSlaveI2C_RSV_08
		case SpektrumSlaveI2C_RSV_08 :
		break;
#endif

#ifdef 	SpektrumSlaveI2C_RSV_09
		case SpektrumSlaveI2C_RSV_09 :
		break;
#endif

#ifdef	SpektrumSlaveI2C_PBOX
		case SpektrumSlaveI2C_PBOX :
			SpektrumTelemetry_PowerBox.volt1 = swap_uint16(battVoltage);   //*1e2
			SpektrumTelemetry_PowerBox.capacity1 = swap_uint16((uint16_t)battCurrentConsumed);
			SpektrumTelemetry_PowerBox.volt2 = 0; // swap_uint16(NotUsed);   //*1e2
			SpektrumTelemetry_PowerBox.capacity2 = 0; //swap_uint16(NotUsed);
			SpektrumTelemetry_PowerBox.alarms = 0;
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_PowerBox, sizeof(SpektrumTelemetry_PowerBox));
		break;
#endif

#ifdef	SpektrumSlaveI2C_AIRSPEED
		case SpektrumSlaveI2C_AIRSPEED :
			SpektrumTelemetry_Speed.airspeed = swap_uint16(airspeed);
			SpektrumTelemetry_Speed.maxAirspeed = swap_uint16(airspeed);
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_Speed, sizeof(SpektrumTelemetry_Speed));
		break;
#endif

#ifdef 	SpektrumSlaveI2C_ALTITUDE
		case SpektrumSlaveI2C_ALTITUDE :
			SpektrumTelemetry_Alt.altitude = swap_int16(altitude * 1e1);	// Value in meters
			SpektrumTelemetry_Alt.maxAltitude = swap_int16(altitude * 1e1);  // Value in meters * 10
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_Alt, sizeof(SpektrumTelemetry_Alt));
		break;
#endif

#ifdef 	SpektrumSlaveI2C_GMETER
		case SpektrumSlaveI2C_GMETER :
		break;
#endif

#ifdef	SpektrumSlaveI2C_JETCAT
		case SpektrumSlaveI2C_JETCAT :
		break;
#endif

#ifdef	SpektrumSlaveI2C_GPS_LOC
		case SpektrumSlaveI2C_GPS_LOC :
			SpektrumTelemetry_GpsLoc.HDOP = decToBcd(gps_hdop);
			SpektrumTelemetry_GpsLoc.course = (uint16_t)dec_ToBcd(gps_course / 10);
			SpektrumTelemetry_GpsLoc.altitudeLow = (uint16_t)dec_ToBcd(gps_alt / 100);
			if (gps_fix){
				convertGPSCoord(&flaggps, &deglat, &minlat, &seclat, &subSeclat, &deglon, &minlon, &seclon, &subSeclon, gps_lat, gps_lon);

				SpektrumTelemetry_GpsLoc.subSecLat = decToBcd((uint8_t)subSeclat);
				SpektrumTelemetry_GpsLoc.degSecLat = decToBcd((uint8_t)seclat);
				SpektrumTelemetry_GpsLoc.degMinLat = decToBcd((uint8_t)minlat);
				SpektrumTelemetry_GpsLoc.degLat = decToBcd((uint8_t)deglat);

				SpektrumTelemetry_GpsLoc.subSecLon = decToBcd((uint8_t)subSeclon);
				SpektrumTelemetry_GpsLoc.degSecLon = decToBcd((uint8_t)seclon);
				SpektrumTelemetry_GpsLoc.degMinLon = decToBcd((uint8_t)minlon);
				SpektrumTelemetry_GpsLoc.degLon = decToBcd((uint8_t)deglon);

				SpektrumTelemetry_GpsLoc.GPSflags = flaggps;
			}

			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_GpsLoc, sizeof(SpektrumTelemetry_GpsLoc));
		break;
#endif

#ifdef 	SpektrumSlaveI2C_GPS_STATS
		case SpektrumSlaveI2C_GPS_STATS :
			SpektrumTelemetry_GpsStat.numSats = decToBcd(gps_sats);
			SpektrumTelemetry_GpsStat.altitudeHigh = decToBcd((uint8_t)0);
			SpektrumTelemetry_GpsStat.UTC_HH = decToBcd(tmGpsTime.Hour);	//Note this is time from start of APM... fix require to APM & mavlink
			SpektrumTelemetry_GpsStat.UTC_MM = decToBcd(tmGpsTime.Minute);
			SpektrumTelemetry_GpsStat.UTC_SS = decToBcd(tmGpsTime.Second);
			SpektrumTelemetry_GpsStat.UTC_MS = decToBcd((uint8_t) 0);
			SpektrumTelemetry_GpsStat.speed = (uint16_t)dec_ToBcd((gps_speed / 100) * 1.94384F); // value is in m/s convert to knots
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_GpsStat, sizeof(SpektrumTelemetry_GpsStat));
		break;
#endif

#ifdef 	SpektrumSlaveI2C_ENERGY_DUAL
		case SpektrumSlaveI2C_ENERGY_DUAL :
		break;
#endif

#ifdef 	SpektrumSlaveI2C_JETCAT_2
		case SpektrumSlaveI2C_JETCAT_2 :
		break;
#endif

#ifdef	SpektrumSlaveI2C_GYRO
		case SpektrumSlaveI2C_GYRO :

		break;
#endif

#ifdef SpektrumSlaveI2C_ATTMAG
		case SpektrumSlaveI2C_ATTMAG :
			SpektrumTelemetry_AttMag.attRoll = swap_int16((int)roll);
			SpektrumTelemetry_AttMag.attPitch = swap_int16((int)pitch);
			SpektrumTelemetry_AttMag.attYaw = swap_int16((int)yaw);
			SpektrumTelemetry_AttMag.magX = 0;
			SpektrumTelemetry_AttMag.magY = 0;
			SpektrumTelemetry_AttMag.magZ = 0;
			break;
#endif

#ifdef 	SpektrumSlaveI2C_AS3X_LEGACYGAIN
		case SpektrumSlaveI2C_AS3X_LEGACYGAIN :
		break;
#endif

#ifdef 	SpektrumSlaveI2C_ESC
		case SpektrumSlaveI2C_ESC :
		break;
#endif

#ifdef 	SpektrumSlaveI2C_FUEL
		case SpektrumSlaveI2C_FUEL :
		break;
#endif

#ifdef SpektrumSlaveI2C_MAH
		case SpektrumSlaveI2C_MAH :
			SpektrumTelemetry_MAH.sID = 0;
			SpektrumTelemetry_MAH.current_A = (180 * 1e1);
			SpektrumTelemetry_MAH.chargeUsed_A = (1000);
			SpektrumTelemetry_MAH.current_B = NotUsed; //(10 * 1e1);
			SpektrumTelemetry_MAH.chargeUsed_B = NotUsed; //(600);
			SpektrumTelemetry_MAH.temp_A = NotUsed;
			SpektrumTelemetry_MAH.temp_B = NotUsed;
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_MAH, sizeof(SpektrumTelemetry_MAH));
		break;
#endif

#ifdef SpektrumSlaveI2C_RPM
		case SpektrumSlaveI2C_RPM :
			SpektrumTelemetry_RPM.volts = swap_uint16(3.3 * 1e2);
			SpektrumTelemetry_RPM.temperature = swap_int16(75);
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_RPM, sizeof(SpektrumTelemetry_RPM));
		break;

#endif

#ifdef SpektrumSlaveI2C_LIPOMON
		case SpektrumSlaveI2C_LIPOMON :
			SpektrumTelemetry_LipoMon.cell_1 = swap_uint16(3.7);
			SpektrumTelemetry_LipoMon.cell_2 = swap_uint16(3.6);
			SpektrumTelemetry_LipoMon.cell_3 = swap_uint16(3.5);
			SpektrumTelemetry_LipoMon.cell_4 = swap_uint16(3.4);
			SpektrumTelemetry_LipoMon.temp = swap_uint16(74);
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_LipoMon, sizeof(SpektrumTelemetry_LipoMon));
			break;
#endif

#ifdef SpektrumSlaveI2C_VARIO_S
		case SpektrumSlaveI2C_VARIO_S :
			SpektrumTelemetry_Vario_S.altitude = swap_int16(50 * 1e1);
			SpektrumTelemetry_Vario_S.delta_0250ms = swap_int16(10);
			SpektrumTelemetry_Vario_S.delta_0500ms = swap_int16(20);
			SpektrumTelemetry_Vario_S.delta_1000ms = swap_int16(30);
			SpektrumTelemetry_Vario_S.delta_1500ms = swap_int16(40);
			SpektrumTelemetry_Vario_S.delta_2000ms = swap_int16(50);
			SpektrumTelemetry_Vario_S.delta_3000ms = swap_int16(60);
			memcpy(&SpektrumTelemetryBuffer, &SpektrumTelemetry_Vario_S, sizeof(SpektrumTelemetry_Vario_S));
			break;

#endif



		default :
			break;
	}
	Wire.write(SpektrumTelemetryBuffer,sizeof(SpektrumTelemetryBuffer));	// Write out telemetry buffer onto i2c bus
	memset(&SpektrumTelemetryBuffer,0,sizeof(SpektrumTelemetryBuffer));		// Clear out telemetry buffer for next i2c address
}

