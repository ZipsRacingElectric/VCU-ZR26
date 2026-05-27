// Header
#include "tv_const_bias.h"

tvOutput_t tvConstBias (const tvInput_t* input, const void* configPointer, void* statePointer)
{
	const tvConstBiasConfig_t* config = configPointer;
	(void) statePointer;

	// Determine which biases to use based on whether the request is for driving torque or regen torque.

	bool drivingBiases = input->torqueRequest >= 0;

	float biasRear  = drivingBiases ? input->drivingFrBias         : input->regenFrBias;
	float biasLeft  = drivingBiases ? config->drivingLeftRightBias : config->regenLeftRightBias;
	float biasFront = 1 - biasRear;
	float biasRight = 1 - biasLeft;

	// Output is the biased torque request.

	tvOutput_t output =
	{
		.valid = true,

		.torqueRl	= input->torqueRequest * biasRear  * biasLeft,
		.torqueRr	= input->torqueRequest * biasRear  * biasRight,
		.torqueFl	= input->torqueRequest * biasFront * biasLeft,
		.torqueFr	= input->torqueRequest * biasFront * biasRight,
	};
	return output;
}