// Header
#include "tv_const_bias.h"

tvOutput_t tvConstBias (const tvInput_t* input, const void* configPointer)
{
	const tvConstBiasConfig_t* config = configPointer;

	// Driving torque bias

	float drivingRearBias	= config->drivingFrontRearBias;
	float drivingFrontBias	= 1 - drivingRearBias;
	float drivingLeftBias	= config->drivingLeftRightBias;
	float drivingRightBias	= 1 - drivingLeftBias;

	// Regen torque bias

	float regenRearBias		= config->regenFrontRearBias;
	float regenFrontBias	= 1 - regenRearBias;
	float regenLeftBias		= config->regenLeftRightBias;
	float regenRightBias	= 1 - regenLeftBias;

	// Output is the biased sum of driving torque and regen torque.

	tvOutput_t output =
	{
		.valid = true,

		.torqueRl	= input->drivingTorqueLimit	* drivingRearBias	* drivingLeftBias
					- input->regenTorqueLimit	* regenRearBias		* regenLeftBias,

		.torqueRr	= input->drivingTorqueLimit * drivingRearBias	* drivingRightBias
					- input->regenTorqueLimit	* regenRearBias		* regenRightBias,

		.torqueFl	= input->drivingTorqueLimit * drivingFrontBias	* drivingLeftBias
					- input->regenTorqueLimit	* regenFrontBias	* regenLeftBias,

		.torqueFr	= input->drivingTorqueLimit	* drivingFrontBias	* drivingRightBias
					- input->regenTorqueLimit	* regenFrontBias	* regenRightBias
	};
	return output;
}