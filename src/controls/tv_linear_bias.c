// Header
#include "tv_linear_bias.h"

// Includes
#include "peripherals.h"
#include "controls/lerp.h"

tvOutput_t tvLinearBias (const tvInput_t* input, const void* configPointer, void* statePointer)
{
	const tvLinearBiasConfig_t* config = configPointer;
	(void) statePointer;

	// Determine which biases to use based on whether the request is for driving torque or regen torque.

	bool drivingBiases = input->torqueRequest >= 0;

	float biasRear  = drivingBiases ? input->drivingFrBias : input->regenFrBias;
	float biasFront = 1 - biasRear;

	float biasLeft;
	float biasRight;

	if (drivingBiases)
	{
		// Linearly interpolate from beginning angle & 50% bias to end angle & end bias (for both positive and negative).

		float steeringAngle = sas.value;

		if (steeringAngle >= 0.0f)
		{
			biasLeft = lerp2dSaturated (sas.value,
				config->steeringAngleBiasBegin, 0.5f,
				config->steeringAngleBiasEnd, config->leftRightBiasEnd);
		}
		else
		{
			biasLeft = lerp2dSaturated (-sas.value,
				config->steeringAngleBiasBegin, 0.5f,
				config->steeringAngleBiasEnd, 1.0f - config->leftRightBiasEnd);
		}

		biasRight = 1.0f - biasLeft;
	}
	else
	{
		// 50:50 left/right bias for regen.
		biasLeft  = 0.5f;
		biasRight = 0.5f;
	}

	// Output is the biased torque request.

	tvOutput_t output =
	{
		.valid = sas.state == ANALOG_SENSOR_VALID,

		.torqueRl	= input->torqueRequest * biasRear  * biasLeft,
		.torqueRr	= input->torqueRequest * biasRear  * biasRight,
		.torqueFl	= input->torqueRequest * biasFront * biasLeft,
		.torqueFr	= input->torqueRequest * biasFront * biasRight,
	};
	return output;
}