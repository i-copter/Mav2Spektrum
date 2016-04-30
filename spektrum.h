/*
 * spektrum.h
 *
 *  Created on: Apr 17, 2016
 *      Author: i-copter@gmail.com
 */

#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_


//////////////////////////////////////////////////////////////////
//
// Copyright 2013 by Horizon Hobby, Inc.
// All Rights Reserved Worldwide.
//
// This header file may be incorporated into non-Horizon
// products.
//
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
//
// Assigned I2C Addresses and Device Types
//
//////////////////////////////////////////////////////////////////

#define SpektrumSlaveI2C_NODATA (0x00) // No data in packet
//#define SpektrumSlaveI2C_VOLTAGE (0x01) // High-Voltage sensor (INTERNAL)
//#define SpektrumSlaveI2C_TEMPERATURE (0x02) // Temperature Sensor (INTERNAL)

#define SpektrumSlaveI2C_IHIGH_CURRENT (0x03) // High-Current sensor (150A)
//#define SpektrumSlaveI2C_RSV_04 (0x04) // Reserved
//#define SpektrumSlaveI2C_RSV_05 (0x05) // Reserved
//#define SpektrumSlaveI2C_RSV_06 (0x06) // Reserved
//#define SpektrumSlaveI2C_RSV_07 (0x07) // Reserved
//#define SpektrumSlaveI2C_RSV_08 (0x08) // Reserved
//#define SpektrumSlaveI2C_RSV_09 (0x09) // Reserved

#define SpektrumSlaveI2C_PBOX (0x0a) // PowerBox
#define SpektrumSlaveI2C_AIRSPEED (0x11) // Air Speed
#define SpektrumSlaveI2C_ALTITUDE (0x12) // Altitude
//#define SpektrumSlaveI2C_GMETER (0x14) // GForce
//#define SpektrumSlaveI2C_JETCAT (0x15) // JetCat interface
#define SpektrumSlaveI2C_GPS_LOC (0x16) // GPS Location Data
#define SpektrumSlaveI2C_GPS_STATS (0x17) // GPS Status
//#define SpektrumSlaveI2C_ENERGY_DUAL (0x18) // Dual Coulomb counter
//#define SpektrumSlaveI2C_JETCAT_2 (0x19) // JetCat interface, msg 2
//#define SpektrumSlaveI2C_GYRO (0x1A) // 3-axis gyro
#define SpektrumSlaveI2C_ATTMAG (0x1B) // Attitude and Magnetic Compass
//#define SpektrumSlaveI2C_AS3X_LEGACYGAIN (0x1F) // Active AS3X Gains for legacy mode
//#define SpektrumSlaveI2C_ESC (0x20) // ESC
//#define SpektrumSlaveI2C_FUEL (0x22) // Fuel Flow Meter

// DO NOT USE (0x30) // Reserved for internal use
// DO NOT USE (0x32) // Reserved for internal use

//#define SpektrumSlaveI2C_MAH (0x34) // Battery Gauge (mAh)
//#define SpektrumSlaveI2C_DIGITAL_AIR (0x36) // Digital Inputs & Tank Pressure
//#define SpektrumSlaveI2C_STRAIN (0x38) // Thrust/Strain Gauge
//#define SpektrumSlaveI2C_LIPOMON (0x3A) // Cell Monitor (LiPo taps)
//#define SpektrumSlaveI2C_VARIO_S (0x40) // Vario
//#define SpektrumSlaveI2C_RSV_43 (0x43) // Reserved
//#define SpektrumSlaveI2C_USER_16SU (0x50) // User-Def, STRU_TELE_USER_16SU
//#define SpektrumSlaveI2C_USER_16SU32U (0x52) // User-Def, STRU_TELE_USER_16SU32U
//#define SpektrumSlaveI2C_USER_16SU32S (0x54) // User-Def, STRU_TELE_USER_16SU32S
//#define SpektrumSlaveI2C_USER_16U32SU (0x56) // User-Def, STRU_TELE_USER_16U32SU
//#define SpektrumSlaveI2C_RSV_60 (0x60) // Reserved
//#define SpektrumSlaveI2C_RSV_68 (0x68) // Reserved
//#define SpektrumSlaveI2C_RSV_69 (0x69) // Reserved
//#define SpektrumSlaveI2C_RSV_6A (0x6A) // Reserved
//#define SpektrumSlaveI2C_RSV_6B (0x6B) // Reserved
//#define SpektrumSlaveI2C_RSV_6C (0x6C) // Reserved
//#define SpektrumSlaveI2C_RSV_6D (0x6D) // Reserved
//#define SpektrumSlaveI2C_RSV_6E (0x6E) // Reserved
//#define SpektrumSlaveI2C_RSV_6F (0x6F) // Reserved
//#define SpektrumSlaveI2C_RSV_70 (0x70) // Reserved
//#define SpektrumSlaveI2C_FRAMEDATA (0x7D) // Transmitter frame data
//#define SpektrumSlaveI2C_RPM (0x7E) // RPM sensor
//#define SpektrumSlaveI2C_QOS (0x7F) // RxV + flight log data

//
// Spektrum I2c Defaults
//
#define SpektrumSlaveI2C_MAX (0x7f) // Last address available
#define SpektrumSlaveI2C_SHORTRANGE (0x80) // Data is from a TM1100
#define NotUsed (0x7fff)

/////////////////////////////////////////////////
//
//
//  Power Box definitions:
//
//
////////////////////////////////////////////////
#define TELE_PBOX_ALARM_VOLTAGE_1 (0x01)
#define TELE_PBOX_ALARM_VOLTAGE_2 (0x02)
#define TELE_PBOX_ALARM_CAPACITY_1 (0x04)
#define TELE_PBOX_ALARM_CAPACITY_2 (0x08)
//#define TELE_PBOX_ALARM_RPM (0x10)
//#define TELE_PBOX_ALARM_TEMPERATURE (0x20)
#define TELE_PBOX_ALARM_RESERVED_1 (0x40)
#define TELE_PBOX_ALARM_RESERVED_2 (0x80)


/////////////////////////////////////////////////
//
//
//  GPS flags definitions:
//
//
////////////////////////////////////////////////
#define GPS_INFO_FLAGS_IS_NORTH_BIT (0)
#define GPS_INFO_FLAGS_IS_NORTH (1 << GPS_INFO_FLAGS_IS_NORTH_BIT)
#define GPS_INFO_FLAGS_IS_EAST_BIT (1)
#define GPS_INFO_FLAGS_IS_EAST (1 << GPS_INFO_FLAGS_IS_EAST_BIT)
#define GPS_INFO_FLAGS_LONG_GREATER_99_BIT (2)
#define GPS_INFO_FLAGS_LONG_GREATER_99 (1 << GPS_INFO_FLAGS_LONG_GREATER_99_BIT)
#define GPS_INFO_FLAGS_GPS_FIX_VALID_BIT (3)
#define GPS_INFO_FLAGS_GPS_FIX_VALID (1 << GPS_INFO_FLAGS_GPS_FIX_VALID_BIT)
#define GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT (4)
#define GPS_INFO_FLAGS_GPS_DATA_RECEIVED (1 << GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT)
#define GPS_INFO_FLAGS_3D_FIX_BIT (5)
#define GPS_INFO_FLAGS_3D_FIX (1 << GPS_INFO_FLAGS_3D_FIX_BIT)
#define GPS_INFO_FLAGS_NEGATIVE_ALT_BIT (7)
#define GPS_INFO_FLAGS_NEGATIVE_ALT (1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT)

#define IHIGH_RESOLUTION_FACTOR ((FP32)(0.196791))

//////////////////////////////////////////////////////////////////
//
// Message Data typedef structures
//
//////////////////////////////////////////////////////////////////


#ifdef SpektrumSlaveI2C_ESC
//////////////////////////////////////////////////////////////////
//
// Electronic Speed Control
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_ESC; // Source device = 0x20
	uint8_t sID; // Secondary ID
	uint16_t RPM; // RPM, 10RPM (0-655340 RPM). 0xFFFF --> "No data"
	uint16_t voltsInput; // Volts, 0.01v (0-655.34V). 0xFFFF --> "No data"
	uint16_t tempFET; // Temperature, 0.1C (0-999.8C) 0xFFFF --> "No data"
	uint16_t currentMotor; // Current, 10mA (0-655.34A). 0xFFFF --> "No data"
	uint16_t tempBEC; // Temperature, 0.1C (0-999.8C) 0x7FFF --> "No data"
	uint8_t currentBEC; // BEC Current, 100mA (0-25.4A). 0xFF ----> "No data"
	uint8_t voltsBEC; // BEC Volts, 0.05V (0-12.70V). 0xFF ----> "No data"
	uint8_t throttle; // 0.5% (0-127%). 0xFF ----> "No data"
	uint8_t powerOut; // Power Output, 0.5% (0-127%). 0xFF ----> "No data"
} SpektrumTelemetry_ESC;
#endif

#ifdef SpektrumSlaveI2C_FUEL
//////////////////////////////////////////////////////////////////
//
// (Liquid) Fuel Flow/Capacity (Two Tanks/Engines)
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_FUEL; // Source device = 0x2
	uint8_t sID; // Secondary ID
	uint16_t fuelConsumed_A; // Integrated fuel consumption, 0.1mL
	uint16_t flowRate_A; // Instantaneous consumption, 0.01mL/min
	uint16_t temp_A; // Temperature, 0.1C (0-655.34C)
	uint16_t fuelConsumed_B; // Integrated fuel consumption, 0.1mL
	uint16_t flowRate_B; // Instantaneous consumption, 0.01mL/min
	uint16_t temp_B; // Temperature, 0.1C (0-655.34C)
	uint16_t spare; // Not used
} SpektrumTelemetry_Fuel;
#endif

#ifdef SpektrumSlaveI2C_MAH
//////////////////////////////////////////////////////////////////
//
// Battery Current/Capacity (Dual Batteries)
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_MAH; // Source device = 0x34
	uint8_t sID; // Secondary ID
	int16_t current_A; // Instantaneous current, 0.1A (0-3276.8A)
	int16_t chargeUsed_A; // Integrated mAh used, 1mAh (0-32.766Ah)
	uint16_t temp_A; // Temperature, 0.1C (0-150.0C,
	// 0x7FFF indicates not populated)
	int16_t current_B; // Instantaneous current, 0.1A (0-6553.4A)
	int16_t chargeUsed_B; // Integrated mAh used, 1mAh (0-65.534Ah)
	uint16_t temp_B; // Temperature, 0.1C (0-150.0C,
	// 0x7FFF indicates not populated)
	uint16_t spare; // Not used
} SpektrumTelemetry_MAH;
#endif

#ifdef SpektrumSlaveI2C_DIGITAL_AIR
//////////////////////////////////////////////////////////////////
//
// Digital Input Status (Retract Status) and Tank Pressure
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_DIGITAL_AIR; // Source device = 0x36
	uint8_t sID; // Secondary ID
	uint16_t digital; // Digital inputs (bit per input)
	uint16_t pressure; // Tank pressure, 0.1PSI (0-6553.4PSI)
} SpektrumTelemetry_DigitalAir;
#endif

#ifdef SpektrumSlaveI2C_STRAIN
//////////////////////////////////////////////////////////////////
//
// Thrust/Strain Gauge
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_STRAIN; // Source device = 0x38
	uint8_t sID; // Secondary ID
	uint16_t strain_A, // Strain sensor A
			strain_B, // Strain sensor B
			strain_C, // Strain sensor D
			strain_D; // Strain sensor C
} SpektrumTelemetry_Strain;
#endif

#ifdef SpektrumSlaveI2C_LIPOMON
//////////////////////////////////////////////////////////////////
//
// LiPo Cell Monitor
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_LIPOMON; // Source device = 0x3A
	uint8_t sID; // Secondary ID
	uint16_t cell_1, // Voltage across cell 1
			cell_2, // Voltage across cell 2
			cell_3, // Voltage across cell 3
			cell_4, // Voltage across cell 4
			cell_5, // Voltage across cell 5
			cell_6; // Voltage across cell 6
	uint16_t temp; // Temperature, 0.1C (0-655.34C)
} SpektrumTelemetry_LipoMon;
#endif

#ifdef SpektrumSlaveI2C_USER_16SU
//////////////////////////////////////////////////////////////////
//
// THIRD-PARTY 16-BIT DATA SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_USER_16SU; // Source device = 0x50
	uint8_t sID; // Secondary ID
	int16_t sField1, // Signed 16-bit data fields
			sField2,
			sField3;
	uint16_t uField1, // Unsigned 16-bit data fields
			uField2,
			uField3,
			uField4;
} SpektrumTelemetry_Usr_16SU;
#endif

#ifdef SpektrumSlaveI2C_USER_16SU32U
//////////////////////////////////////////////////////////////////
//
// THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT UNSIGNED
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_USER_16SU32U; // Source device = 0x52
	uint8_t sID; // Secondary ID
	int16_t sField1, // Signed 16-bit data fields
			sField2;
	uint16_t uField1, // Unsigned 16-bit data fields
			uField2,
			uField3;
	long u32Field; // Unsigned 32-bit data field
} SpektrumTelemetry_Usr_16SU32U;
#endif

#ifdef SpektrumSlaveI2C_USER_16SU32S
//////////////////////////////////////////////////////////////////
//
// THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT SIGNED
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_USER_16SU32S; // Source device = 0x54
	uint8_t sID; // Secondary ID
	int16_t sField1, // Signed 16-bit data fields
			sField2;
	uint16_t uField1, // Unsigned 16-bit data fields
			uField2,
			uField3;
	long u32Field; // Signed 32-bit data field
} SpektrumTelemetry_Usr_16U32S;
#endif

#ifdef SpektrumSlaveI2C_USER_16U32SU
//////////////////////////////////////////////////////////////////
//
// THIRD-PARTY 16-BIT UNSIGNED AND 32-BIT SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_USER_16U32SU; // Source device = 0x56
	uint8_t sID; // Secondary ID
	uint16_t uField1; // Unsigned 16-bit data field
	long u32Field; // Signed 32-bit data field
	long u32Field1, // Signed 32-bit data fields
		u32Field2;
} SpektrumTelemetry_Usr_16U32SU;
#endif

#ifdef SpektrumSlaveI2C_PBOX
//////////////////////////////////////////////////////////////////
//
// POWERBOX
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_PBOX; // Source device = 0x0a
	uint8_t sID; // Secondary ID
	uint16_t volt1; // Volts, 0v01v
	uint16_t volt2; // Volts, 0.01v
	uint16_t capacity1; // mAh, 1mAh
	uint16_t capacity2; // mAh, 1mAh
	uint16_t spare16_1;
	uint16_t spare16_2;
	uint8_t spare;
	uint8_t alarms; // Alarm bitmask (see below)
} SpektrumTelemetry_PowerBox;
#endif

#ifdef SpektrumSlaveI2C_ENERGY_DUAL
//////////////////////////////////////////////////////////////////
//
// DUAL ENERGY
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_ENERGY_DUAL; // Source device = 0x18
	uint8_t sID; // Secondary ID
	int16_t current_A; // Instantaneous current, 0.01A (0-328.7A)
	int16_t chargeUsed_A; // Integrated mAh used, 0.1mAh (0-3276.6mAh)
	uint16_t volts_A; // Voltage, 0.01VC (0-16.00V)
	int16_t current_B; // Instantaneous current, 0.1A (0-3276.8A)
	int16_t chargeUsed_B; // Integrated mAh used, 1mAh (0-32.766Ah)
	uint16_t volts_B; // Voltage, 0.01VC (0-16.00V)
	uint16_t spare; // Not used
} SpektrumTelemetry_Energy_Dual;
#endif

#ifdef SpektrumSlaveI2C_IHIGH_CURRENT
//////////////////////////////////////////////////////////////////
//
// HIGH-CURRENT
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_IHIGH_CURRENT; // Source device = 0x03
	uint8_t sID; // Secondary ID
	int16_t current, // Range: +/- 150A
	// Resolution: 300A/2048 = 0.196791 A/tick
	dummy; // TBD
} SpektrumTelemetry_IHigh;
#endif

#ifdef SpektrumSlaveI2C_VARIO_S
//////////////////////////////////////////////////////////////////
//
// VARIO-S
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_VARIO_S; // Source device = 0x40
	uint8_t sID; // Secondary ID
	int16_t altitude; // .1m increments
	int16_t delta_0250ms, // delta last 250ms, 0.1m/s increments
			delta_0500ms, // delta last 500ms, 0.1m/s increments
			delta_1000ms, // delta last 1.0 seconds
			delta_1500ms, // delta last 1.5 seconds
			delta_2000ms, // delta last 2.0 seconds
			delta_3000ms; // delta last 3.0 seconds
} SpektrumTelemetry_Vario_S;
#endif

#ifdef SpektrumSlaveI2C_ALTITUDE
//////////////////////////////////////////////////////////////////
//
// ALTIMETER
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_ALTITUDE;  // Source device = 0x12
	uint8_t sID; // Secondary ID
	int16_t altitude; // .1m increments
	int16_t maxAltitude; // .1m increments
} SpektrumTelemetry_Alt;
#endif

#ifdef SpektrumSlaveI2C_AIRSPEED
////////////////////////////////////////////////////////////////
//
// AIRSPEED
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_AIRSPEED;  // Source device = 0x11
	uint8_t sID; // Secondary ID
	uint16_t airspeed; // 1 km/h increments
	uint16_t maxAirspeed; // 1 km/h increments
} SpektrumTelemetry_Speed;
#endif

#ifdef SpektrumSlaveI2C_GMETER
//////////////////////////////////////////////////////////////////
//
// GFORCE
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_GMETER; // Source device = 0x14
	uint8_t sID; // Secondary ID
	int16_t GForceX; // force is reported as .01G increments
	int16_t GForceY; // Range = +/-4000 (+/- 40G) in Pro model
	int16_t GForceZ; // Range = +/-800 (+/- 8G) in Standard model
	int16_t maxGForceX; // abs(max G X-axis) FORE/AFT
	int16_t maxGForceY; // abs (max G Y-axis) LEFT/RIGHT
	int16_t maxGForceZ; // max G Z-axis WING SPAR LOAD
	int16_t minGForceZ; // min G Z-axis WING SPAR LOAD
} SpektrumTelemetry_GMeter;
#endif

#ifdef SpektrumSlaveI2C_JETCAT
//////////////////////////////////////////////////////////////////
//
// JETCAT/TURBINE
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_JETCAT; // Source device = 0x15
	uint8_t sID; // Secondary ID
	uint8_t status; // See table below
	uint8_t throttle; // (BCD) xx Percent
	uint16_t packVoltage; // (BCD) xx.yy
	uint16_t pumpVoltage; // (BCD) xx.yy
	uint16_t RPM_MSB;  // (BCD)
	uint16_t RPM_LSB;
	uint16_t EGT; // (BCD) Temperature, Celsius
	uint8_t offCondition; // (BCD) See table below
	uint8_t spare;
} SpektrumTelemetry_JetCat;


enum JETCAT_ECU_TURBINE_STATE { // ECU Status definitions
JETCAT_ECU_STATE_OFF = 0x00,
JETCAT_ECU_STATE_WAIT_for_RPM = 0x01, // (Stby/Start)
JETCAT_ECU_STATE_Ignite = 0x02,
JETCAT_ECU_STATE_Accelerate = 0x03,
JETCAT_ECU_STATE_Stabilise = 0x04,
JETCAT_ECU_STATE_Learn_HI = 0x05,
JETCAT_ECU_STATE_Learn_LO = 0x06,
JETCAT_ECU_STATE_UNDEFINED = 0x07,
JETCAT_ECU_STATE_Slow_Down = 0x08,
JETCAT_ECU_STATE_Manual = 0x09,
JETCAT_ECU_STATE_AutoOff = 0x10,
JETCAT_ECU_STATE_Run = 0x11, // (reg.)
JETCAT_ECU_STATE_Accleleration_delay = 0x12,
JETCAT_ECU_STATE_SpeedReg = 0x13, // (Speed Ctrl)
JETCAT_ECU_STATE_Two_Shaft_Regulate = 0x14, // (only for secondary shaft)
JETCAT_ECU_STATE_PreHeat1 = 0x15,
JETCAT_ECU_STATE_PreHeat2 = 0x16,
JETCAT_ECU_STATE_MainFStart = 0x17,
JETCAT_ECU_STATE_NotUsed = 0x18,
JETCAT_ECU_STATE_KeroFullOn = 0x19,
// undefined states 0x1A-0x1F
EVOJET_ECU_STATE_off = 0x20,
EVOJET_ECU_STATE_ignt = 0x21,
EVOJET_ECU_STATE_acce = 0x22,
EVOJET_ECU_STATE_run = 0x23,
EVOJET_ECU_STATE_cal = 0x24,
EVOJET_ECU_STATE_cool = 0x25,
EVOJET_ECU_STATE_fire = 0x26,
EVOJET_ECU_STATE_glow = 0x27,
EVOJET_ECU_STATE_heat = 0x28,
EVOJET_ECU_STATE_idle = 0x29,
EVOJET_ECU_STATE_lock = 0x2A,
EVOJET_ECU_STATE_rel = 0x2B,
EVOJET_ECU_STATE_spin = 0x2C,
EVOJET_ECU_STATE_stop = 0x2D,
// undefined states 0x2E-0x2F
HORNET_ECU_STATE_OFF = 0x30,
HORNET_ECU_STATE_SLOWDOWN = 0x31,
HORNET_ECU_STATE_COOL_DOWN = 0x32,
HORNET_ECU_STATE_AUTO = 0x33,
HORNET_ECU_STATE_AUTO_HC = 0x34,
HORNET_ECU_STATE_BURNER_ON = 0x35,
HORNET_ECU_STATE_CAL_IDLE = 0x36,
HORNET_ECU_STATE_CALIBRATE = 0x37,
HORNET_ECU_STATE_DEV_DELAY = 0x38,
HORNET_ECU_STATE_EMERGENCY = 0x39,
HORNET_ECU_STATE_FUEL_HEAT = 0x3A,
HORNET_ECU_STATE_FUEL_IGNITE = 0x3B,
HORNET_ECU_STATE_GO_IDLE = 0x3C,
HORNET_ECU_STATE_PROP_IGNITE = 0x3D,
HORNET_ECU_STATE_RAMP_DELAY = 0x3E,
HORNET_ECU_STATE_RAMP_UP = 0x3F,
HORNET_ECU_STATE_STANDBY = 0x40,
HORNET_ECU_STATE_STEADY = 0x41,
HORNET_ECU_STATE_WAIT_ACC = 0x42,
HORNET_ECU_STATE_ERROR = 0x43,
// undefined states 0x44-0x4F
XICOY_ECU_STATE_Temp_High = 0x50,
XICOY_ECU_STATE_Trim_Low = 0x51,
XICOY_ECU_STATE_Set_Idle = 0x52,
XICOY_ECU_STATE_Ready = 0x53,
XICOY_ECU_STATE_Ignition = 0x54,
XICOY_ECU_STATE_Fuel_Ramp = 0x55,
XICOY_ECU_STATE_Glow_Test = 0x56,
XICOY_ECU_STATE_Running = 0x57,
XICOY_ECU_STATE_Stop = 0x58,
XICOY_ECU_STATE_Flameout = 0x59,
XICOY_ECU_STATE_Speed_Low = 0x5A,
XICOY_ECU_STATE_Cooling = 0x5B,
XICOY_ECU_STATE_Igniter_Bad = 0x5C,
XICOY_ECU_STATE_Starter_F = 0x5D,
XICOY_ECU_STATE_Weak_Fuel = 0x5E,
XICOY_ECU_STATE_Start_On = 0x5F,
XICOY_ECU_STATE_Pre_Heat = 0x60,
XICOY_ECU_STATE_Battery = 0x61,
XICOY_ECU_STATE_Time_Out = 0x62,
XICOY_ECU_STATE_Overload = 0x63,
XICOY_ECU_STATE_Igniter_Fail = 0x64,
XICOY_ECU_STATE_Burner_On = 0x65,
XICOY_ECU_STATE_Starting = 0x66,
XICOY_ECU_STATE_SwitchOver = 0x67,
XICOY_ECU_STATE_Cal_Pump = 0x68,
XICOY_ECU_STATE_Pump_Limit = 0x69,
XICOY_ECU_STATE_No_Engine = 0x6A,
XICOY_ECU_STATE_Pwr_Boost = 0x6B,
XICOY_ECU_STATE_Run_Idle = 0x6C,
XICOY_ECU_STATE_Run_Max = 0x6D,
TURBINE_ECU_MAX_STATE = 0x74
} ;

enum JETCAT_ECU_OFF_CONDITIONS { // ECU off conditions. Valid only when the
ECUStatus = JETCAT_ECU_STATE_OFF,
JETCAT_ECU_OFF_No_Off_Condition_defined = 0,
JETCAT_ECU_OFF_Shut_down_via_RC,
JETCAT_ECU_OFF_Overtemperature,
JETCAT_ECU_OFF_Ignition_timeout,
JETCAT_ECU_OFF_Acceleration_time_out,
JETCAT_ECU_OFF_Acceleration_too_slow,
JETCAT_ECU_OFF_Over_RPM,
JETCAT_ECU_OFF_Low_Rpm_Off,
JETCAT_ECU_OFF_Low_Battery,
JETCAT_ECU_OFF_Auto_Off,
JETCAT_ECU_OFF_Low_temperature_Off,
JETCAT_ECU_OFF_Hi_Temp_Off,
JETCAT_ECU_OFF_Glow_Plug_defective,
JETCAT_ECU_OFF_Watch_Dog_Timer,
JETCAT_ECU_OFF_Fail_Safe_Off,
JETCAT_ECU_OFF_Manual_Off, // (via GSU)
JETCAT_ECU_OFF_Power_fail, // (Battery fail)
JETCAT_ECU_OFF_Temp_Sensor_fail, // (only during startup)
JETCAT_ECU_OFF_Fuel_fail,
JETCAT_ECU_OFF_Prop_fail,
JETCAT_ECU_OFF_2nd_Engine_fail,
JETCAT_ECU_OFF_2nd_Engine_Diff_Too_High,
JETCAT_ECU_OFF_2nd_Engine_No_Comm,
JETCAT_ECU_MAX_OFF_COND
} ;
#endif

#ifdef SpektrumSlaveI2C_JETCAT_2
struct
{
	const uint8_t id = SpektrumSlaveI2C_JETCAT_2; // Source device = 0x19
	uint8_t sID; // Secondary ID
	uint16_t FuelFlowRateMLMin; // (BCD) mL per Minute
	uint16_t RestFuelVolumeInTankML_MSB;	// (BCD) mL remaining in tank
	uint16_t RestFuelVolumeInTankML_LSB;
	// 8 bytes left
} SpektrumTelemetry_JetCat2;
#endif

#ifdef SpektrumSlaveI2C_GPS_LOC
//////////////////////////////////////////////////////////////////
//
// GPS
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_GPS_LOC; // Source device = 0x16
	uint8_t sID; // Secondary ID
	uint16_t altitudeLow; // BCD, meters, format 3.1 (Low bits of alt)
	uint16_t latitude_MSB;	// BCD, format 4.4
	uint16_t latitude_LSB;
	// Degrees * 100 + minutes, < 100 degrees
	uint16_t longitude_MSB;	// BCD, format 4.4
	uint16_t longitude_LSB;
	// Degrees * 100 + minutes, flag --> > 99deg
	uint16_t course; // BCD, 3.1
	uint8_t HDOP; // BCD, format 1.1
	uint8_t GPSflags; // see definitions below
} SpektrumTelemetry_GpsLoc;
#endif

#ifdef SpektrumSlaveI2C_GPS_STATS
struct
{
	const uint8_t id = SpektrumSlaveI2C_GPS_STATS; // Source device = 0x17
	uint8_t sID; // Secondary ID
	uint16_t speed; // BCD, knots, format 3.1
	//UINT32 UTC; // BCD, format HH:MM:SS.S, format 6.1
	uint8_t	UTC_MS;
	uint8_t UTC_SS;
	uint8_t	UTC_MM;
	uint8_t UTC_HH;
	uint8_t numSats; // BCD, 0-99
	uint8_t altitudeHigh; // BCD, meters, format 2.0 (High bits alt)
} SpektrumTelemetry_GpsStat;
#endif

#ifdef SpektrumSlaveI2C_GYRO
//////////////////////////////////////////////////////////////////
//
// GYRO
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_GYRO; // Source device = 0x1A
	uint8_t sID; // Secondary ID
	int16_t gyroX; // Rotation rates of the body - Rate
	// is about the X Axis which is
	// defined out the nose of the
	// vehicle.
	int16_t gyroY; // Units are 0.1 deg/sec - Rate is
	// about the Y Axis which is defined
	// out the right wing of the vehicle.
	int16_t gyroZ; // Rate is about the Z axis which is
	// defined down from the vehicle.
	int16_t maxGyroX; // Max rates (absolute value)
	int16_t maxGyroY;
	int16_t maxGyroZ;
} SpektrumTelemetry_Gyro;
#endif

#ifdef SpektrumSlaveI2C_ATTMAG
//////////////////////////////////////////////////////////////////
//
// ATTITUDE & MAG COMPASS
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_ATTMAG; // Source device = 0x1B
	uint8_t sID; // Secondary ID
	int16_t attRoll; // Attitude, 3 axes. Roll is a
	// rotation about the X Axis of
	// the vehicle using the RHR.
	int16_t attPitch; // Units are 0.1 deg - Pitch is a
	// rotation about the Y Axis of the
	// vehicle using the RHR.
	int16_t attYaw; // Yaw is a rotation about the Z
	// Axis of the vehicle using the RHR.
	int16_t magX; // Magnetic Compass, 3 axes
	int16_t magY; // Units are TBD
	int16_t magZ; //
} SpektrumTelemetry_AttMag;
#endif

#ifdef SpektrumSlaveI2C_FRAMEDATA
//////////////////////////////////////////////////////////////////
//
// Transmitter Frame Data
//
//////////////////////////////////////////////////////////////////
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_FRAMEDATA; // Source device = 0x7D
	uint8_t sID; // Secondary ID
	uint16_t chanData[7]; // Channel Data array
} SpektrumTelemetry_FrameData;
#endif

#ifdef SpektrumSlaveI2C_RPM
//////////////////////////////////////////////////////////////////
//
// RPM/Volts/Temperature
//
//////////////////////////////////////////////////////////////////
//

struct
{
	const uint8_t id = SpektrumSlaveI2C_RPM; // Source device = 0x7E
	uint8_t sID; // Secondary ID
	uint16_t microseconds; // microseconds between pulse leading edges
	uint16_t volts; // 0.01V increments
	int16_t temperature; // degrees F
} SpektrumTelemetry_RPM;
#endif

#ifdef SpektrumSlaveI2C_QOS
//////////////////////////////////////////////////////////////////
//
// QoS DATA
//
//////////////////////////////////////////////////////////////////
//
// NOTE: AR6410-series send:
// id = 7F
// sID = 0
// A = 0
// B = 0
// L = 0
// R = 0
// F = fades
// H = holds
// rxV = 0xFFFF
//
struct
{
	const uint8_t id = SpektrumSlaveI2C_QOS; // Source device = 0x7F
	uint8_t sID; // Secondary ID
	uint16_t A;
	uint16_t B;
	uint16_t L;
	uint16_t R;
	uint16_t F;
	uint16_t H;
	uint16_t rxVoltage; // Volts, 0.01V increments
} SpektrumTelemetry_QOS;
#endif



//////////////////////////////////////////////////////////////////
//
// UNION OF ALL DEVICE MESSAGES
//
//////////////////////////////////////////////////////////////////
//

//typedef union
//{
//uint16_t raw[8];
//SpektrumTelemetry_QOS qos;
//SpektrumTelemetry_RPM rpm;
//SpektrumTelemetry_FrameData frame;
//SpektrumTelemetry_Alt alt;
//SpektrumTelemetry_Speed speed;
//SpektrumTelemetry_Energy_Dual eDual;
//SpektrumTelemetry_Vario_S varioS;
//SpektrumTelemetry_GMeter accel;
//SpektrumTelemetry_JetCat jetcat;
//SpektrumTelemetry_JetCat2 jetcat2;
//SpektrumTelemetry_GpsLoc gpsloc;
//SpektrumTelemetry_GpsStat gpsstat;
//SpektrumTelemetry_Gyro gyro;
//SpektrumTelemetry_AttMag attMag;
//SpektrumTelemetry_PowerBox powerBox;
//SpektrumTelemetry_ESC escGeneric;
//SpektrumTelemetry_Fuel fuel;
//SpektrumTelemetry_MAH mAh;
//SpektrumTelemetry_DigitalAir digAir;
//SpektrumTelemetry_Strain strain;
//SpektrumTelemetry_LipoMon lipomon;
//SpektrumTelemetry_Usr_16SU user_16SU;
//SpektrumTelemetry_Usr_16SU32U user_16SU32U;
//SpektrumTelemetry_Usr_16U32S user_16U32S;
//SpektrumTelemetry_Usr_16U32SU user_16U32SU;
//} UN_TELEMETRY; // All telemetry messages




#endif /* SPEKTRUM_H_ */
