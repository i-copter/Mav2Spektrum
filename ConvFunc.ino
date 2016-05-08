/*
 * ConvFunc.ino
 *
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
 *  Created on: Apr 17, 2016
 *      Author: i-copter.com
 *
 *      GPS Coordinate code from mav2telem
 *      Menno de Gans (joebar.rc@googlemail.com)
 *
 *
 */

#include "Mav2Spektrum.h"


uint8_t decToBcd(uint8_t val)
{
  return (((val/10)<<4) + (val%10));
}

uint16_t decToBcd(uint16_t val)
{
	return (((val/100)<<8) + (val%100));

}

uint32_t dec_ToBcd(uint16_t dec)
{
    return (dec) ? ((dec_ToBcd( dec / 10 ) << 4) + (dec % 10)) : 0;
}

uint16_t swap_uint16( uint16_t val )
{
    return (val << 8) | (val >> 8 );
}

//! Byte swap short
int16_t swap_int16( int16_t val )
{
    return (val << 8) | ((val >> 8) & 0xFF);
}

//! Byte swap unsigned int
uint32_t swap_uint32( uint32_t val )
{
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | (val >> 16);
}

//! Byte swap int
int32_t swap_int32( int32_t val )
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

void convertGPSCoord(int8_t *gpsflag, int8_t *latdeg, int8_t *latmin, int8_t *latsec, int8_t *latsubSec, int8_t *londeg, int8_t *lonmin, int8_t *lonsec, int8_t *lonsubSec, int32_t coordlat, int32_t coordlon){
	int32_t minPart;
	int32_t secPart;
	int32_t subSecPart;

  if(coordlat < 0){
    coordlat=-coordlat;
  }

  else {
	  *latdeg=coordlat/1E7;
	  minPart=coordlat - *latdeg * 1E7;
	  *latmin=(minPart * 60)/1E7;
	  secPart=minPart*60 - *latmin * 1E7;
	  *latsec=(secPart * 100)/1E7;
	  subSecPart=secPart*100 - *latsec * 1E7;
	  *latsubSec=(subSecPart * 100)/1E7;
	  *gpsflag |= 0x01;
  }

  if (coordlon < 0){
	  coordlon=-coordlon;
  }
  else {
	  *londeg=coordlon/1E7;
	  minPart=coordlon - *londeg * 1E7;
	  *lonmin=(minPart * 60)/1E7;
	  secPart=minPart*60 - *lonmin * 1E7;
	  *lonsec=(secPart * 100)/1E7;
	  subSecPart=secPart*100 - *lonsec * 1E7;
	  *lonsubSec=(subSecPart * 100)/1E7;
	  *gpsflag |= 0x02;
	  if (*londeg > 99){
		  *gpsflag |= 0x04;
		  *londeg -= 100;
	  }
  }
  if (gps_fix)
	  *gpsflag |= 0x10;
  else
	  *gpsflag &= 0xef;

return;
}
