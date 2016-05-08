# Mav2Spektrum

Mavlink protocol to Spektrum TM1000 telemetry module

This project converts mavlink protocol to Spektrums i2c telemetry TM1000 module using a Teensy 3.2 32-bit Arduino compatable module from PJRC.  This version does not rely on the FastSerial implementation and only uses the native serial I/O class.

Serial connection to mavlink is via pins 0(RX1)
i2c connection to Spektrum TM1000 module is via 19(SCL) and 18(SDA)

Pinouts of the Teensy module are at http://www.pjrc.com/teensy
The Spektrum Telemetry protocol and pinouts can be found at http://www.spektrumrc.com/ProdInfo/Files/SPM_Telemetry_Developers_Specs.pdf

Change Serial1 to Serial for USB connectivity instead of hardware serial port... useful for interfaces with USB such as the RaspberryPi flight controllers (Navio+, Navio2, Erie, etc...)


Known Issues / TODO:

1. Fix gps time -- currently mavlink return a 64bit value -- Fixed.. however ArduPilot returns the time since boot...
2. Altitude needs some work


GPS Coordinate code from mav2telem
Menno de Gans (joebar.rc@googlemail.com)


