#ifndef EEPROM_MAP_H
#define EEPROM_MAP_H

// EEPROM Mapping -------------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.10.22
//
// Description: Structing mapping the data of an EEPROM data to variables.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "peripherals/pedals.h"
#include "peripherals/steering_angle.h"
#include "controls/tv_const_bias.h"
#include "controls/tv_linear_bias.h"
#include "controls/tv_bicycle_model_tucker.h"

// Constants ------------------------------------------------------------------------------------------------------------------

/// @brief The magic string of the EEPROM. Update this value every time the memory map changes to force manual re-programming.
#define EEPROM_MAP_STRING "VCU_2026_05_25"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	uint8_t pad0 [16];								// 0x0000

	pedalsConfig_t pedalConfig;						// 0x0010

	float drivingTorqueLimit;						// 0x0030
	float regenTorqueLimit;							// 0x0034
	float drivingFrBias;							// 0x0038
	float regenFrBias;								// 0x003C
	bool sasEnabled;								// 0x0040
	bool amkAutoResetRequest;						// 0x0041
	uint16_t torqueAlgoritmIndex;					// 0x0042

	uint16_t glvBattery20v;							// 0x0044
	uint16_t glvBattery29v;							// 0x0046

	sasConfig_t sasConfig;							// 0x0048

	float regenDeratingThrottleStart;				// 0x005C
	float regenDeratingThrottleEnd;					// 0x0060
	float regenDeratingSpeedLow;					// 0x0064
	float regenDeratingSpeedHigh;					// 0x0068
	float regenDeratingHysteresis;					// 0x006C

	float powerLimit;								// 0x0070
	float powerLimitPidKp;							// 0x0074
	float powerLimitPidKi;							// 0x0078
	float powerLimitPidKd;							// 0x007C
	float powerLimitPidA;							// 0x0080

	// Straight-diff TV config
	tvConstBiasConfig_t sdConfig;					// 0x0084

	// Linear-steering TV config
	tvLinearBiasConfig_t lsConfig;					// 0x008C

	bool watchdogEnabled;							// 0x0098

	// 0x009C to 0x00B4

	/// @brief The reduction ratio of the vehicle's gearboxes. Ex, 14:1 => 14, 27:2 => 13.5.
	float gearRatio;
	/// @brief The radius of the vehicle's wheels, in inches.
	float wheelRadius;
	/// @brief The distance between the center of the front wheel and the center of the rear wheel, along the x-axis, in
	/// meters.
	float wheelBase;
	/// @brief The front-to-rear bias of the vehicle's weight distribution. 0 => 100% rearwards, 1 => 100% frontwards.
	float frontRearWeightBias;
	/// @brief The distance between the center of the front left and front right wheels' contact patches, in meters.
	float trackWidthRear;
	/// @brief The distance between the center of the rear left and rear right wheels' contact patches, in meters.
	float trackWidthFront;
	/// @brief The reduction ratio of the vehicle's steering rack. Ex, 10 => 110 deg steering wheel angle = 11 deg wheel angle.
	float steeringRatio;
	/// @brief The moment of intertia of the vehicle, about the yaw axis, in kg*m^2.
	float yawMomentOfInertia;

	uint8_t pad1 [68];								// 0x00B4

	tvBicycleModelTuckerConfig_t bicycleConfig;		// 0x0100
} eepromMap_t;

// Functions ------------------------------------------------------------------------------------------------------------------

bool eepromReadonlyRead (void* object, uint16_t addr, void* data, uint16_t dataCount);

bool eepromWriteonlyWrite (void* object, uint16_t addr, const void* data, uint16_t dataCount);

#endif // EEPROM_MAP_H