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

// Constants ------------------------------------------------------------------------------------------------------------------

/// @brief The magic string of the EEPROM. Update this value every time the memory map changes to force manual re-programming.
#define EEPROM_MAP_STRING "VCU_2026_04_21"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	uint8_t pad0 [16];							// 0x0000

	pedalsConfig_t pedalConfig;					// 0x0010

	float drivingTorqueLimit;					// 0x0030
	float regenTorqueLimit;						// 0x0034
	float drivingFrBias;						// 0x0038
	float regenFrBias;							// 0x003C
	bool sasEnabled;							// 0x0040
	bool amkAutoResetRequest;					// 0x0041
	uint16_t torqueAlgoritmIndex;				// 0x0042

	uint16_t glvBattery20v;						// 0x0044
	uint16_t glvBattery29v;						// 0x0046

	sasConfig_t sasConfig;						// 0x0048

	float regenDeratingThrottleStart;			// 0x005C
	float regenDeratingThrottleEnd;				// 0x0060
	float regenDeratingSpeedEnd;				// 0x0064
	float regenDeratingSpeedStart;				// 0x0068

	float powerLimit;							// 0x006C
	float powerLimitPidKp;						// 0x0070
	float powerLimitPidKi;						// 0x0074
	float powerLimitPidKd;						// 0x0078
	float powerLimitPidA;						// 0x007C

	// Straight-diff TV config
	tvConstBiasConfig_t sdConfig;				// 0x0080

	// Linear-steering TV config
	tvLinearBiasConfig_t lsConfig;				// 0x0088
} eepromMap_t;

// Functions ------------------------------------------------------------------------------------------------------------------

bool eepromReadonlyRead (void* object, uint16_t addr, void* data, uint16_t dataCount);

bool eepromWriteonlyWrite (void* object, uint16_t addr, const void* data, uint16_t dataCount);

#endif // EEPROM_MAP_H