#ifndef TV_BICYCLE_MODEL_TUCKER_H
#define TV_BICYCLE_MODEL_TUCKER_H

// Abby Tucker's Torque Vectoring Bicycle Model -------------------------------------------------------------------------------
//
// Author: Abby Tucker, Cole Barach
// Date Created: 2026.05.20
//
// Description: TODO(Barach)
//
// References:
// TODO(Barach): Still relevant?
// - https://www.youtube.com/watch?v=coHGv3G2JOU

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "torque_vectoring.h"
#include "controls/pid_controller.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	float kp;
	float ki;
	float kd;
	float ka;
	float frontRearMomentBias;
} tvBicycleModelTuckerConfig_t;

typedef struct
{
	pidController_t pid;
	float xdPrime;
} tvBicycleModelTuckerState_t;

// Functions ------------------------------------------------------------------------------------------------------------------

/// @brief Entrypoint to the torque vectoring algorithm.
tvOutput_t tvBicycleModelTucker (const tvInput_t* input, const void* configPointer, void* statePointer);

#endif // TV_BICYCLE_MODEL_TUCKER_H