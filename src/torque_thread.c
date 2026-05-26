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
#include "controls/lerp.h"

// C Standard Library
#include <float.h>

// Constants ------------------------------------------------------------------------------------------------------------------

#define TORQUE_THREAD_PERIOD TIME_MS2I (10)
#define TORQUE_THREAD_PERIOD_S (TIME_I2US (TORQUE_THREAD_PERIOD) / 1000000.0f)
#define TORQUE_THREAD_CAN_MESSAGE_TIMEOUT (TORQUE_THREAD_PERIOD / 4)

#define CUMULATIVE_TORQUE_TOLERANCE		0.05f

#define SIB_TORQUE_LIMIT_INDEX			2
#define SIB_REGEN_LIMIT_INDEX			0
#define SIB_REGEN_BIAS_INDEX			1

#define SIB_UP_INDEX					6
#define SIB_DOWN_INDEX					5

#define SIB_TORQUE_STEP					3.0f
#define SIB_BIAS_STEP					0.05

// Global Data ----------------------------------------------------------------------------------------------------------------

tvOutput_t nonDeratedOutput;

float drivingTorqueLimit = 0.0f;
float regenTorqueLimit = 0.0f;
float drivingFrontRearBias = 0.5f;
float regenFrontRearBias = 0.5f;

static uint8_t algoritmIndex = 0;

static tvBicycleModelTuckerState_t tvBicycleModelTuckerState =
{
	.pid = { 0 },
	.xdPrime = 0
};

static regenState_t regenRlState = { 0 };
static regenState_t regenRrState = { 0 };
static regenState_t regenFlState = { 0 };
static regenState_t regenFrState = { 0 };

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

/// @brief  PID controller responsible for enforcing the global power limit.
powerLimiter_t powerLimiter;

// Function Prototypes --------------------------------------------------------------------------------------------------------

/**
 * @brief Calculates the input structure to pass to the selected torque-vectoring algorithm.
 * @param deltaTime The amount of time that has passed since the last call to this function.
 * @return The calculated input structure.
 */
static tvInput_t requestCalculateInput (float deltaTime);

/**
 * @brief Executes the selected torque-vectoring algorithm, calculating an amount of torque to request of each inverter.
 * @note This function only calculates the torque request.
 * @param input The input structure, as returned by @c requestCalculateInput .
 * @return The output structure containing the torque to request.
 */
static tvOutput_t requestCalculateOutput (tvInput_t* input);

bool requestApplyDerating (tvOutput_t* output, tvInput_t* input);

/**
 * @brief Checks the validity of a torque request.
 * @param output The request to validate.
 * @param input The input structure that generated said request.
 * @return True if the request is valid, false otherwise.
 */
static bool requestValidate (tvOutput_t* output, tvInput_t* input);

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

		// Steering input board controls
		if (sibGetButtonHeldLock (&steeringInputBoard, SIB_TORQUE_LIMIT_INDEX))
		{
			// Adjust driving torque limit
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_UP_INDEX))
				torqueThreadSetDrivingTorqueLimit (drivingTorqueLimit + SIB_TORQUE_STEP);
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_DOWN_INDEX))
				torqueThreadSetDrivingTorqueLimit (drivingTorqueLimit - SIB_TORQUE_STEP);
		}
		else if (sibGetButtonHeldLock (&steeringInputBoard, SIB_REGEN_LIMIT_INDEX))
		{
			// Adjust regen torque limit
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_UP_INDEX))
				torqueThreadSetRegenTorqueLimit (regenTorqueLimit + SIB_TORQUE_STEP);
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_DOWN_INDEX))
				torqueThreadSetRegenTorqueLimit (regenTorqueLimit - SIB_TORQUE_STEP);
		}
		else if (sibGetButtonHeldLock (&steeringInputBoard, SIB_REGEN_BIAS_INDEX))
		{
			// Adjust regen F/R bias
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_UP_INDEX))
				torqueThreadSetRegenFrBias (regenFrontRearBias - SIB_BIAS_STEP);
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_DOWN_INDEX))
				torqueThreadSetRegenFrBias (regenFrontRearBias + SIB_BIAS_STEP);
		}
		else
		{
			// Adjust driving F/R bias
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_UP_INDEX))
				torqueThreadSetDrivingFrBias (drivingFrontRearBias - SIB_BIAS_STEP);
			if (sibGetButtonDownLock (&steeringInputBoard, SIB_DOWN_INDEX))
				torqueThreadSetDrivingFrBias (drivingFrontRearBias + SIB_BIAS_STEP);
		}

		// Sample the sensor inputs.
		peripheralsSample (timePrevious, timeCurrent);

		bool resetRequest = physicalEepromMap->amkAutoResetRequest;
		tvInput_t input = requestCalculateInput (TORQUE_THREAD_PERIOD_S);
		tvOutput_t output = requestCalculateOutput (&input);
		nonDeratedOutput = output;

		bool derating = requestApplyDerating (&output, &input);
		bool plausible = requestValidate (&output, &input);

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

		// Transmit the non-derated torque message (used for debugging).
		transmitNonderatedTorqueMessage (&CAND1, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
	}
}

// Functions ------------------------------------------------------------------------------------------------------------------

void torqueThreadStart (tprio_t priority)
{
	// Start the torque control thread
	chThdCreateStatic (&torqueThreadWa, sizeof (torqueThreadWa), priority, torqueThread, NULL);
}

void torqueThreadSelectAlgorithm (uint8_t index)
{
	// Clamp index
	if (index < TV_ALGORITHM_COUNT)
		algoritmIndex = index;
}

void torqueThreadSetDrivingTorqueLimit (float torque)
{
	if (torque > AMK_DRIVING_TORQUE_MAX * AMK_COUNT)
	{
		drivingTorqueLimit = AMK_DRIVING_TORQUE_MAX * AMK_COUNT;
		return;
	}

	if (torque < 0)
	{
		drivingTorqueLimit = 0;
		return;
	}

	drivingTorqueLimit = torque;
}

void torqueThreadSetRegenTorqueLimit (float torque)
{
	// Allow both positive and negative values, as I will probably enter values incorrectly.
	if (torque > 0)
		torque = -torque;

	// Clamp to the max magnitude
	if (torque < AMK_REGENERATIVE_TORQUE_MAX * AMK_COUNT)
	{
		regenTorqueLimit = AMK_REGENERATIVE_TORQUE_MAX * AMK_COUNT;
		return;
	}

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
	if (bias < 0)
		bias = 0;
	if (bias > 1)
		bias = 1;

	drivingFrontRearBias = bias;
}

void torqueThreadSetRegenFrBias (float bias)
{
	if (bias < 0)
		bias = 0;
	if (bias > 1)
		bias = 1;

	regenFrontRearBias = bias;
}

tvInput_t requestCalculateInput (float deltaTime)
{
	// Regen input
	float paddleInput0 = sibGetAnalogValueLock (&steeringInputBoard, 0);
	float paddleInput1 = sibGetAnalogValueLock (&steeringInputBoard, 1);
	float paddleRequest = paddleInput0 > paddleInput1 ? paddleInput0 : paddleInput1;

	// Derate based on throttle position (no regen when throttle is depressed)
	paddleRequest = lerp2dSaturated (pedals.appsRequest,
		physicalEepromMap->regenDeratingThrottleLow, paddleRequest,
		physicalEepromMap->regenDeratingThrottleHigh, 0);

	// Calculate power limiting
	float power = bmsGetPowerLock (&bms);
	float limitRatio = powerLimiterCalculateTorqueLimit (&powerLimiter, power, deltaTime);

	// Calculate regen derating
	float regenTorqueLimitRl = regenLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkRl, &regenRlState);
	float regenTorqueLimitRr = regenLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkRr, &regenRrState);
	float regenTorqueLimitFl = regenLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkFl, &regenFlState);
	float regenTorqueLimitFr = regenLimit (AMK_REGENERATIVE_TORQUE_MAX, &amkFr, &regenFrState);

	// Torque request is the combination of the throttle request and regen request, scaled by the power limiting ratio.
	float torqueRequest = (pedals.appsRequest * drivingTorqueLimit + paddleRequest * regenTorqueLimit) * limitRatio;

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

tvOutput_t requestCalculateOutput (tvInput_t* input)
{
	// Use the user-specified torque-vectoring algorithm to calculate the torque request.
	return tvAlgorithms [algoritmIndex].entrypoint (input, tvAlgorithms [algoritmIndex].config, tvAlgorithms [algoritmIndex].state);
}

bool requestApplyDerating (tvOutput_t* output, tvInput_t* input)
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

bool requestValidate (tvOutput_t* output, tvInput_t* input)
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