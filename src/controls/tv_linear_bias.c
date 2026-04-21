// Header
#include "tv_linear_bias.h"

// Includes
#include "peripherals.h"
#include "controls/lerp.h"

tvOutput_t tvLinearBias (const tvInput_t* input, const void* configPointer)
{
	const tvLinearBiasConfig_t* config = configPointer;

	// Constant front-to-rear bias

	float drivingBiasRear = config->drivingFrontRearBias;
	float drivingBiasFront = 1.0f - drivingBiasRear;

	float regenBiasRear = config->regenFrontRearBias;
	float regenBiasFront = 1.0f - regenBiasRear;

	// Linearly interpolate from beginning angle & 50% bias to end angle & end bias (for both positive and negative).

	float steeringAngle = sas.value;
	float biasLeft;
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
	float biasRight = 1.0f - biasLeft;

	// Output is the biased sum of driving torque and regen torque.

	tvOutput_t output =
	{
		.valid = sas.state == ANALOG_SENSOR_VALID,

		.torqueRl =   drivingBiasRear	* biasLeft	* input->drivingTorqueLimit
					- regenBiasRear		* 0.5f		* input->regenTorqueLimit,

		.torqueRr =   drivingBiasRear	* biasRight	* input->drivingTorqueLimit
					- regenBiasRear		* 0.5f		* input->regenTorqueLimit,

		.torqueFl =   drivingBiasFront	* biasLeft	* input->drivingTorqueLimit
					- regenBiasFront	* 0.5f		* input->regenTorqueLimit,

		.torqueFr =   drivingBiasFront	* biasRight	* input->drivingTorqueLimit
					- regenBiasFront	* 0.5f		* input->regenTorqueLimit
	};
	return output;
}