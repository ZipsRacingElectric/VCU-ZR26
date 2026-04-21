#ifndef TV_CONST_BIAS
#define TV_CONST_BIAS

// Constant Bias Torque Vectoring Algorithm -----------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.10.12
//
// Description: Bare minimum torque vectoring algorithm. This is only intended as a fallback for when critical sensors are not
//   working, please don't run this unless you are desperate.
//
// Sensor Requirements: None
//
// Algorithm Description:
// - Applies a constant front-to-rear & left-to-right bias to the requested amount of torque.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "torque_vectoring.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	/// @brief The left-to-right bias of the driving torque (1 => 100% left, 0 => 100% right).
	float drivingLeftRightBias;
	/// @brief The left-to-right bias of the regen torque (1 => 100% left, 0 => 100% right).
	float regenLeftRightBias;
} tvConstBiasConfig_t;

// Functions ------------------------------------------------------------------------------------------------------------------

/// @brief Entrypoint to the torque vectoring algorithm.
tvOutput_t tvConstBias (const tvInput_t* input, const void* configPointer, void* statePointer);

#endif // TV_CONST_BIAS