/*
 * ConvFunc.ino
 *
 *  Created on: Apr 17, 2016
 *      Author: i-copter.com
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


