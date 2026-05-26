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

/// @brief Input to torque vectoring algorithms.
typedef struct
{
	/// @brief The amount of time elapsed between now and the previous call to the algorithm's update function, in seconds.
	float deltaTime;

	/// @brief The total amount of torque the algorithm is to provide. Positive for driving torque, negative for regen torque.
	/// While it should be respected if possible, the actual torques request may sum to a magnitude less than this due to
	/// saturation.
	float torqueRequest;

	/// @brief The maximum requestable amount of driving torque to the RL motor. Any more will be saturated.
	float drivingTorqueLimitRl;
	/// @brief The maximum requestable amount of driving torque to the RR motor. Any more will be saturated.
	float drivingTorqueLimitRr;
	/// @brief The maximum requestable amount of driving torque to the FL motor. Any more will be saturated.
	float drivingTorqueLimitFl;
	/// @brief The maximum requestable amount of driving torque to the FR motor. Any more will be saturated.
	float drivingTorqueLimitFr;

	/// @brief The maximum requestable amount of regen torque to the RL motor. Any more will be saturated.
	float regenTorqueLimitRl;
	/// @brief The maximum requestable amount of regen torque to the RR motor. Any more will be saturated.
	float regenTorqueLimitRr;
	/// @brief The maximum requestable amount of regen torque to the FL motor. Any more will be saturated.
	float regenTorqueLimitFl;
	/// @brief The maximum requestable amount of regen torque to the FR motor. Any more will be saturated.
	float regenTorqueLimitFr;

	/// @brief The front-to-rear bias for distributing driving torque. Not required for all torque vectoring algorithms.
	/// 1 => 100% rearwards, 0 => 100% frontwards, 0.5 => 50:50 split.
	float drivingFrBias;

	/// @brief The front-to-rear bias for distributing regen torque. Not required for all torque vectoring algorithms.
	/// 1 => 100% rearwards, 0 => 100% frontwards, 0.5 => 50:50 split.
	float regenFrBias;
} tvInput_t;

/// @brief Output of torque vectoring algorithms.
typedef struct
{
	/// @brief Output request validity. Indicates whether the the output of this algoritm is usable.
	bool valid;

	/// @brief The torque to request to the rear-left motor. Positive for driving torque, negative for regen torque.
	float torqueRl;

	/// @brief The torque to request to the rear-right motor. Positive for driving torque, negative for regen torque.
	float torqueRr;

	/// @brief The torque to request to the front-left motor. Positive for driving torque, negative for regen torque.
	float torqueFl;

	/// @brief The torque to request to the front-right motor. Positive for driving torque, negative for regen torque.
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