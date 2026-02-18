#ifndef TV_BICYCLE_MODEL_H
#define TV_BICYCLE_MODEL_H

// References:
// - https://www.youtube.com/watch?v=coHGv3G2JOU

// Includes
#include "torque_vectoring.h"

typedef struct
{
	/// @brief The distance between the front and rear wheels, in meters.
	float wheelBase;
	/// @brief The proportional gain of the controller.
	float gain;
	/// @brief The front-to-rear bias of the base torque request.
	float biasFr;
} tvBicycleModelConfig_t;

/// @brief Entrypoint to the torque vectoring algorithm.
tvOutput_t tvBicycleModel (const tvInput_t* input, const void* configPointer);

#endif // TV_BICYCLE_MODEL_H