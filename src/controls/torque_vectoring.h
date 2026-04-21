#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

// Torque Vectoring -----------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.10.12
//
// Description: Datatypes and constants common to all torque vectoring algorithms.

// Includes -------------------------------------------------------------------------------------------------------------------

// C Standard Library
#include <stdbool.h>

// Datatypes ------------------------------------------------------------------------------------------------------------------

/// @brief Input to TV algorithms.
typedef struct
{
	/// @brief The amount of time elapsed between now and the previous call to the algorithm's update function, in seconds.
	float deltaTime;

	/// @brief The maximum amount of cumulative requested driving torque. This value is set by both the driver's configuration
	/// and the current throttle position.
	float drivingTorqueLimit;

	/// @brief The maximum amount of cumulative requested regenerative torque. This value is set by the driver's configuration
	/// and the regen request.
	float regenTorqueLimit;

	/// @brief The front-to-rear bias for distributing driving torque. Not required for all torque vectoring algorithms.
	/// 1 => 100% rearwards, 0 => 100% frontwards, 0.5 => 50:50 split.
	float drivingFrBias;

	/// @brief The front-to-rear bias for distributing regen torque. Not required for all torque vectoring algorithms.
	/// 1 => 100% rearwards, 0 => 100% frontwards, 0.5 => 50:50 split.
	float regenFrBias;
} tvInput_t;

/// @brief Output to TV algoritms.
typedef struct
{
	/// @brief Output request validity. Indicates whether the the output of this algoritm is usable.
	bool valid;

	/// @brief The torque to request to the rear-left motor. Positive means actual torque, negative means regen torque.
	float torqueRl;

	/// @brief The torque to request to the rear-right motor. Positive means actual torque, negative means regen torque.
	float torqueRr;

	/// @brief The torque to request to the front-left motor. Positive means actual torque, negative means regen torque.
	float torqueFl;

	/// @brief The torque to request to the front-right motor. Positive means actual torque, negative means regen torque.
	float torqueFr;
} tvOutput_t;

typedef tvOutput_t (tvFunction_t) (const tvInput_t* input, const void* configPointer, void* statePointer);

typedef struct
{
	tvFunction_t* entrypoint;
	const void* config;
	void* state;
} tvAlgorithm_t;

#endif // TORQUE_VECTORING_H