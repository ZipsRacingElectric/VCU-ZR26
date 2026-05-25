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
 * @brief Derates a regen torque request. This is required for 2 main reasons:
 * - The vehicle cannot use regen below 5 km/h by the FSAE rules.
 * - Wheels that lock up due to regen cannot be allowed to spin in reverse.
 * @note This function is very sensitive to latency, therefore, it should be called immediately before the torque request is
 * sent, no earlier.
 * @param torque The torque request. Positive means driving torque, negative means regenerative torque. This function does
 * nothing if the torque request is positive.
 * @param amk The inverter the torque is to be requested of.
 * @param state Structure to write the function's state into. Cannot be modified by any other functions. Must be initialized to
 * all 0's.
 * @return True if the request was derated, false otherwise.
 */
bool regenDerate (float* torque, amkInverter_t* amk, regenState_t* state);

#endif // REGEN_H