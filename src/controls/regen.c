// Header
#include "regen.h"

// Includes
#include "peripherals.h"
#include "controls/lerp.h"
#include "controls/vehicle_dynamics.h"

float regenLimit (float motorLimit, amkInverter_t* amk, regenState_t* state)
{
	// Get the motor speed
	canNodeLock ((canNode_t*) amk);
	float motorSpeed = amk->actualSpeed;
	canNodeUnlock ((canNode_t*) amk);

	// Calculate the effective ground speed of the wheel based on the motor speed. Note this is not the actual ground speed due
	// to tire slip.
	float groundSpeed = motorSpeedToGroundSpeed (motorSpeed, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius);

	// Calculate the ratio to derate the regen request by. This is based on linear interpolation between the low and high regen
	// speeds. This uses hysteresis to prevent backlash in the gearboxes and instability due to underdampened systems (tire
	// elasticity and motor flux).
	float deratingRatio = inverseLerpHysteresis (groundSpeed, physicalEepromMap->regenDeratingSpeedLow,
		physicalEepromMap->regenDeratingSpeedHigh, physicalEepromMap->regenDeratingHysteresis,
		&state->deratingRatioIncreasing, &state->deratingRatioPrime);

	// The regen limit is the motor's limit scaled by the derating ratio
	return motorLimit * deratingRatio;
}