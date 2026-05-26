// Header
#include "torque_thread.h"

// Includes
#include "can.h"
#include "can/transmit.h"
#include "controls/power_limiting.h"
#include "controls/regen.h"
#include "controls/torque_vectoring.h"
#include "controls/tv_const_bias.h"
#include "controls/tv_linear_bias.h"
#include "controls/tv_bicycle_model_tucker.h"
#include "peripherals.h"
#include "state_thread.h"

// C Standard Library
#include <float.h>

// Constants ------------------------------------------------------------------------------------------------------------------

#define TORQUE_THREAD_PERIOD				TIME_MS2I (10)
#define TORQUE_THREAD_CAN_MESSAGE_TIMEOUT	(TORQUE_THREAD_PERIOD / 6)

#define SIB_TORQUE_LIMIT_INDEX				2
#define SIB_REGEN_LIMIT_INDEX				0
#define SIB_REGEN_BIAS_INDEX				1

#define SIB_UP_INDEX						6
#define SIB_DOWN_INDEX						5

#define SIB_TORQUE_STEP						3.0f
#define SIB_BIAS_STEP						0.05

// Global Data ----------------------------------------------------------------------------------------------------------------

// External
uint8_t torqueAlgoritmIndex = 0;
float drivingTorqueLimit = 0.0f;
float regenTorqueLimit = 0.0f;
float drivingFrontRearBias = 0.5f;
float regenFrontRearBias = 0.5f;
tvOutput_t torqueRequestNonDerated;

// Regen limiting
static regenState_t regenRlState = { 0 };
static regenState_t regenRrState = { 0 };
static regenState_t regenFlState = { 0 };
static regenState_t regenFrState = { 0 };

/// @brief PID controller responsible for enforcing the global power limit.
powerLimiter_t powerLimiter = { 0 };

static tvBicycleModelTuckerState_t tvBicycleModelTuckerState =
{
	.pid = { 0 },
	.xdPrime = 0
};

/// @brief The array of selectable torque-vectoring algorithms.
#define TV_ALGORITHM_COUNT (sizeof (tvAlgorithms) / sizeof (tvAlgorithm_t))
static tvAlgorithm_t tvAlgorithms [] =
{
	{
		// Straight-diff
		.entrypoint	= &tvConstBias,
		.config		= &physicalEepromMap->sdConfig,
		.state		= NULL
	},
	{
		// Linear-steering
		.entrypoint	= &tvLinearBias,
		.config		= &physicalEepromMap->lsConfig,
		.state		= NULL
	},
	{
		// Bicycle model
		.entrypoint	= &tvBicycleModelTucker,
		.config		= &physicalEepromMap->bicycleConfig,
		.state		= &tvBicycleModelTuckerState
	}
};

// Functions ------------------------------------------------------------------------------------------------------------------

static float getSibInputs (void)
{
	canNodeLock ((canNode_t*) &steeringInputBoard);

	if (sibGetButtonHeld (&steeringInputBoard, SIB_TORQUE_LIMIT_INDEX))
	{
		// Adjust driving torque limit
		if (sibGetButtonDown (&steeringInputBoard, SIB_UP_INDEX))
			torqueThreadSetDrivingTorqueLimit (drivingTorqueLimit + SIB_TORQUE_STEP);
		if (sibGetButtonDown (&steeringInputBoard, SIB_DOWN_INDEX))
			torqueThreadSetDrivingTorqueLimit (drivingTorqueLimit - SIB_TORQUE_STEP);
	}
	else if (sibGetButtonHeld (&steeringInputBoard, SIB_REGEN_LIMIT_INDEX))
	{
		// Adjust regen torque limit
		if (sibGetButtonDown (&steeringInputBoard, SIB_UP_INDEX))
			torqueThreadSetRegenTorqueLimit (regenTorqueLimit + SIB_TORQUE_STEP);
		if (sibGetButtonDown (&steeringInputBoard, SIB_DOWN_INDEX))
			torqueThreadSetRegenTorqueLimit (regenTorqueLimit - SIB_TORQUE_STEP);
	}
	else if (sibGetButtonHeld (&steeringInputBoard, SIB_REGEN_BIAS_INDEX))
	{
		// Adjust regen F/R bias
		if (sibGetButtonDown (&steeringInputBoard, SIB_UP_INDEX))
			torqueThreadSetRegenFrBias (regenFrontRearBias - SIB_BIAS_STEP);
		if (sibGetButtonDown (&steeringInputBoard, SIB_DOWN_INDEX))
			torqueThreadSetRegenFrBias (regenFrontRearBias + SIB_BIAS_STEP);
	}
	else
	{
		// Adjust driving F/R bias
		if (sibGetButtonDown (&steeringInputBoard, SIB_UP_INDEX))
			torqueThreadSetDrivingFrBias (drivingFrontRearBias - SIB_BIAS_STEP);
		if (sibGetButtonDown (&steeringInputBoard, SIB_DOWN_INDEX))
			torqueThreadSetDrivingFrBias (drivingFrontRearBias + SIB_BIAS_STEP);
	}

	// Regen input (max of the 2 paddles)
	float paddleInput0 = sibGetAnalogValue (&steeringInputBoard, 0);
	float paddleInput1 = sibGetAnalogValue (&steeringInputBoard, 1);

	canNodeUnlock ((canNode_t*) &steeringInputBoard);

	return paddleInput0 > paddleInput1 ? paddleInput0 : paddleInput1;
}

/**
 * @brief Calculates the input structure to pass to the selected torque-vectoring algorithm.
 * @param timePrevious The last system time this function was called.
 * @param timeCurrent The current system time.
 * @return The calculated input structure.
 */
static tvInput_t requestCalculateInput (systime_t timePrevious, systime_t timeCurrent)
{
	// Calculate the amount of time that has ellapsed.
	float deltaTime = TIME_I2US (chTimeDiffX (timePrevious, timeCurrent)) * 1e-6f;

	// Sample the sensor inputs and get the regen input.
	peripheralsSample (timePrevious, timeCurrent);
	float regenRequest = getSibInputs ();

	// Derate the regen request based on throttle position (no regen when throttle is depressed)
	regenRequest = regenDerateRequest (regenRequest, pedals.appsRequest);

	// Calculate power limiting
	float power = bmsGetPowerLock (&bms);
	float limitRatio = powerLimiterCalculateTorqueLimit (&powerLimiter, power, deltaTime);

	// Calculate regen limiting
	float regenTorqueLimitRl = regenDerateLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkRl, &regenRlState);
	float regenTorqueLimitRr = regenDerateLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkRr, &regenRrState);
	float regenTorqueLimitFl = regenDerateLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkFl, &regenFlState);
	float regenTorqueLimitFr = regenDerateLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkFr, &regenFrState);

	// Torque request is the combination of the throttle request and regen request, scaled by the power limiting ratio.
	float torqueRequest = (pedals.appsRequest * drivingTorqueLimit + regenRequest * regenTorqueLimit) * limitRatio;

	return (tvInput_t)
	{
		.deltaTime				= deltaTime,
		.torqueRequest			= torqueRequest,
		.drivingTorqueLimitRl	= AMK_DRIVING_TORQUE_MAX,
		.drivingTorqueLimitRr	= AMK_DRIVING_TORQUE_MAX,
		.drivingTorqueLimitFl	= AMK_DRIVING_TORQUE_MAX,
		.drivingTorqueLimitFr	= AMK_DRIVING_TORQUE_MAX,
		.regenTorqueLimitRl		= regenTorqueLimitRl,
		.regenTorqueLimitRr		= regenTorqueLimitRr,
		.regenTorqueLimitFl		= regenTorqueLimitFl,
		.regenTorqueLimitFr		= regenTorqueLimitFr,
		.drivingFrBias			= drivingFrontRearBias,
		.regenFrBias			= regenFrontRearBias,
	};
}

/**
 * @brief Executes the selected torque-vectoring algorithm, calculating an amount of torque to request of each inverter.
 * @note This function only calculates the torque request.
 * @param input The input structure, as returned by @c requestCalculateInput .
 * @return The output structure containing the torque to request.
 */
static tvOutput_t requestCalculateOutput (tvInput_t* input)
{
	// Use the user-specified torque-vectoring algorithm to calculate the torque request.
	uint8_t index = torqueAlgoritmIndex;
	return tvAlgorithms [index].entrypoint (input, tvAlgorithms [index].config, tvAlgorithms [index].state);
}

/**
 * @brief Derates a torque request based on the per-motor driving/regen torque limits.
 * @param output The request to derate.
 * @param input The input that generated the request.
 * @return True if the request was derated, false otherwise.
 */
static bool requestApplyDerating (tvOutput_t* output, tvInput_t* input)
{
	bool derated = false;

	if (output->torqueRl > input->drivingTorqueLimitRl)
	{
		output->torqueRl = input->drivingTorqueLimitRl;
		derated = true;
	}
	else if (output->torqueRl < input->regenTorqueLimitRl)
	{
		output->torqueRl = input->regenTorqueLimitRl;
		derated = true;
	}

	if (output->torqueRr > input->drivingTorqueLimitRr)
	{
		output->torqueRr = input->drivingTorqueLimitRr;
		derated = true;
	}
	else if (output->torqueRr < input->regenTorqueLimitRr)
	{
		output->torqueRr = input->regenTorqueLimitRr;
		derated = true;
	}

	if (output->torqueFl > input->drivingTorqueLimitFl)
	{
		output->torqueFl = input->drivingTorqueLimitFl;
		derated = true;
	}
	else if (output->torqueFl < input->regenTorqueLimitFl)
	{
		output->torqueFl = input->regenTorqueLimitFl;
		derated = true;
	}

	if (output->torqueFr > input->drivingTorqueLimitFr)
	{
		output->torqueFr = input->drivingTorqueLimitFr;
		derated = true;
	}
	else if (output->torqueFr < input->regenTorqueLimitFr)
	{
		output->torqueFr = input->regenTorqueLimitFr;
		derated = true;
	}

	return derated;
}

/**
 * @brief Checks the validity of a torque request.
 * @param output The request to validate.
 * @param input The input structure that generated said request.
 * @return True if the request is valid, false otherwise.
 */
static bool requestValidate (tvOutput_t* output, tvInput_t* input)
{
	// Check the algorithm's output validity.
	bool valid = output->valid;

	// Check the pedal plausibility.
	valid &= pedals.plausible;

	// TODO(Barach): Need to rework for medium/high speed.
	(void) input;

	// // Calculate the cumulative driving and regen torques. These are calculated separately, as regen shouldn't allow more than
	// // the max driving torque to be requested, and vice versa.
	// float drivingTorque = 0.0f;
	// float regenTorque = 0.0f;

	// if (output->torqueRl >= 0.0f)
	// 	drivingTorque += output->torqueRl;
	// else
	//  	regenTorque -= output->torqueRl;

	// if (output->torqueRr >= 0.0f)
	// 	drivingTorque += output->torqueRr;
	// else
	//  	regenTorque -= output->torqueRr;;

	// if (output->torqueFl >= 0.0f)
	// 	drivingTorque += output->torqueFl;
	// else
	// 	regenTorque -= output->torqueFl;

	// if (output->torqueFr >= 0.0f)
	// 	drivingTorque += output->torqueFr;
	// else
	//  	regenTorque -= output->torqueFr;

	// // Validate the cumulative torque limits are not exceeded.
	// valid &= drivingTorque <= input->drivingTorqueLimit * (1 + CUMULATIVE_TORQUE_TOLERANCE);
	// valid &= regenTorque >= -input->regenTorqueLimit * (1 + CUMULATIVE_TORQUE_TOLERANCE);

	return valid;
}

// Thread Entrypoint ----------------------------------------------------------------------------------------------------------

static THD_WORKING_AREA (torqueThreadWa, 512);
THD_FUNCTION (torqueThread, arg)
{
	(void) arg;
	chRegSetThreadName ("torque_control");

	systime_t timeCurrent = chVTGetSystemTimeX ();
	while (true)
	{
		// Sleep until next loop.
		systime_t timePrevious = timeCurrent;
		systime_t timeNext = chTimeAddX (timeCurrent, TORQUE_THREAD_PERIOD);
		chThdSleepUntilWindowed (timeCurrent, timeNext);
		timeCurrent = chVTGetSystemTimeX ();

		// Calculate the torque vectoring input then execute the algorithm.
		tvInput_t input = requestCalculateInput (timePrevious, timeCurrent);
		tvOutput_t output = requestCalculateOutput (&input);

		// Copy the request before applying derating (for logging on the CAN bus).
		torqueRequestNonDerated = output;

		// Apply derating and validate the input.
		bool derating = requestApplyDerating (&output, &input);
		bool plausible = requestValidate (&output, &input);

		bool resetRequest = physicalEepromMap->amkAutoResetRequest;

		if (vehicleState == VEHICLE_STATE_READY_TO_DRIVE)
		{
			if (plausible)
			{
				// Send the torque request messages.
				amkSendTorqueRequest (&amkRl, output.torqueRl, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkRr, output.torqueRr, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkFl, output.torqueFl, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkFr, output.torqueFr, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
			}
			else
			{
				// Send 0 torque request message (keep torque limits, as lowering them in motion will trigger a fault).
				amkSendTorqueRequest (&amkRl, 0, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkRr, 0, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkFl, 0, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkFr, 0, AMK_DRIVING_TORQUE_MAX, AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
			}
		}
		else
		{
			// De-energization message.
			amkSendEnergizationRequest (&amkRl, false, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
			amkSendEnergizationRequest (&amkRr, false, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
			amkSendEnergizationRequest (&amkFl, false, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
			amkSendEnergizationRequest (&amkFr, false, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
		}

		// Nofify the state thread of the current plausibility.
		stateThreadSetTorquePlausibility (plausible, derating);

		// Transmit the non-derated torque message (for data logging).
		transmitNonderatedTorqueMessage (&CAND1, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
	}
}

void torqueThreadStart (tprio_t priority)
{
	// Start the torque control thread
	chThdCreateStatic (&torqueThreadWa, sizeof (torqueThreadWa), priority, torqueThread, NULL);
}

void torqueThreadSelectAlgorithm (uint8_t index)
{
	// Apply modulus to index (wraps around)
	if (index < TV_ALGORITHM_COUNT)
		index = index % TV_ALGORITHM_COUNT;

	torqueAlgoritmIndex = index;
}

void torqueThreadSetDrivingTorqueLimit (float torque)
{
	// Clamp to min / max
	if (torque > AMK_DRIVING_TORQUE_MAX * AMK_COUNT)
		torque = AMK_DRIVING_TORQUE_MAX * AMK_COUNT;
	if (torque < 0)
		torque = 0;

	drivingTorqueLimit = torque;
}

void torqueThreadSetRegenTorqueLimit (float torque)
{
	// Allow both positive and negative values, as I will probably enter values incorrectly.
	if (torque > 0)
		torque = -torque;

	// Clamp to the max magnitude
	if (torque < AMK_REGENERATIVE_TORQUE_MAX * AMK_COUNT)
		regenTorqueLimit = AMK_REGENERATIVE_TORQUE_MAX * AMK_COUNT;

	regenTorqueLimit = torque;
}

void torqueThreadUpdatePowerLimiter ()
{
	powerLimiterInit (&powerLimiter,
		physicalEepromMap->powerLimitPidKp,
		physicalEepromMap->powerLimitPidKi,
		physicalEepromMap->powerLimitPidKd,
		physicalEepromMap->powerLimitPidKa,
		physicalEepromMap->powerLimit);
}

void torqueThreadSetDrivingFrBias (float bias)
{
	// Clamp to min/max
	if (bias < 0)
		bias = 0;
	if (bias > 1)
		bias = 1;

	drivingFrontRearBias = bias;
}

void torqueThreadSetRegenFrBias (float bias)
{
	// Clamp to min/max
	if (bias < 0)
		bias = 0;
	if (bias > 1)
		bias = 1;

	regenFrontRearBias = bias;
}