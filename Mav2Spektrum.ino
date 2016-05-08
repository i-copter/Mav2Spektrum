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

#include "Arduino.h"
#include "Mav2Spektrum.h"

void setup() {
		Serial1.begin(TELEMETRY_SPEED);
		Serial.begin(115200);  //Debug Port
		pinMode(13, OUTPUT);
	    spektrumInit();
}


void loop() {

	receive_mavlink();
}






