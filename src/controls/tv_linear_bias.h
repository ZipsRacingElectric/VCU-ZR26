#ifndef TV_SAS_LINEAR_H
#define TV_SAS_LINEAR_H

// Linear Steering Angle Sensor Torque Vectoring Algorithm --------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2025.07.12
//
// Description: A torque vectoring algorithm where the left-to-right bias is a linear function of the vehicle's steering angle.
//   This algorithm is a simple open-loop controller, meant to roughly emulate the behavior of a differential without the
//   complexity of negative feedback.
//
//   Driving torque is distributed with a constant front-to-rear bias. Between the begining and ending steering angle, left-to-
//   right bias is linearly interpolated between 50% and the ending ratio.
//   Regen torque is distributed with a constant front-to-rear bias. No left-to-right biasing is used for regen torque.
//
// Sensor Requirements:
//   - Steering-angle-sensor

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "torque_vectoring.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	/// @brief The front-to-rear bias for driving torque.
	float drivingFrontRearBias;
	/// @brief The front-to-rear bias for regen torque.
	float regenFrontRearBias;

	/// @brief The steering angle at which the left-to-right bias starts shifting from 50/50.
	float steeringAngleBiasBegin;
	/// @brief The steering angle at which the left-to-right bias stops shifting to the end value.
	float steeringAngleBiasEnd;
	/// @brief The left-to-right bias at the ending steering angle.
	float leftRightBiasEnd;
} tvLinearBiasConfig_t;

// Functions ------------------------------------------------------------------------------------------------------------------

/// @brief Entrypoint to the torque vectoring algorithm.
tvOutput_t tvLinearBias (const tvInput_t* input, const void* configPointer);

#endif // TV_SAS_LINEAR_H