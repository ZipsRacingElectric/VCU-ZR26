#ifndef REGEN_H
#define REGEN_H

// Regenerative Braking -------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2026.05.25
//
// Description: Code for managing regenerative braking.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "can/amk_inverter.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	bool deratingRatioIncreasing;
	float deratingRatioPrime;
} regenState_t;

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief TODO(Barach). This is required for 2 main reasons:
 * - The vehicle cannot use regen below 5 km/h by the FSAE rules.
 * - Wheels that lock up due to regen cannot be allowed to spin in reverse.
 * @param motorLimit The maximum amount of torque requestable of the motor, should be negative.
 * @param amk The inverter the torque is to be requested of.
 * @param state Structure to write the function's state into. Cannot be modified by any other functions. Must be initialized to
 * all 0's.
 * @return The regen limit that should be enforced. Always negative.
 */
float regenLimit (float motorLimit, amkInverter_t* amk, regenState_t* state);

#endif // REGEN_H