// Header
#include "torque_thread.h"

// Includes
#include "can.h"
#include "controls/pid_controller.h"
#include "controls/torque_vectoring.h"
#include "controls/tv_const_bias.h"
#include "controls/tv_linear_bias.h"
#include "controls/tv_bicycle_model.h"
#include "peripherals.h"
#include "state_thread.h"
#include "controls/lerp.h"

// C Standard Library
#include <float.h>

// Constants ------------------------------------------------------------------------------------------------------------------

#define TORQUE_THREAD_PERIOD TIME_MS2I (10)
#define TORQUE_THREAD_PERIOD_S (TIME_I2US (TORQUE_THREAD_PERIOD) / 1000000.0f)
#define TORQUE_THREAD_CAN_MESSAGE_TIMEOUT (TORQUE_THREAD_PERIOD / 4)

#define CUMULATIVE_TORQUE_TOLERANCE 0.05f

#define SIB_TORQUE_UP_INDEX			6
#define SIB_TORQUE_DOWN_INDEX		5
#define SIB_TORQUE_STEP				12.0f

// Global Data ----------------------------------------------------------------------------------------------------------------

/// @brief The last calculated torque request.
tvOutput_t torqueRequest;

/// @brief The cumulative driving (positive) torque limit.
float drivingTorqueLimit = 0.0f;

/// @brief The cumulative regenerative (negative) torque limit.
float regenTorqueLimit = 0.0f;

/// @brief The index of the selected torque-vectoring algorithm.
static uint8_t algoritmIndex = 0;

static const tvConstBiasConfig_t STF_L_CONFIG =
{
	.drivingFrontRearBias	= 1,
	.drivingLeftRightBias	= 1,
	.regenFrontRearBias		= 1,
	.regenLeftRightBias		= 1
};

static const tvConstBiasConfig_t STF_R_CONFIG =
{
	.drivingFrontRearBias	= 1,
	.drivingLeftRightBias	= 0,
	.regenFrontRearBias		= 1,
	.regenLeftRightBias		= 0
};

/// @brief The array of selectable torque-vectoring algorithms.
#define TV_ALGORITHM_COUNT (sizeof (tvAlgorithms) / sizeof (tvAlgorithm_t))
static tvAlgorithm_t tvAlgorithms [] =
{
	{
		// Straight-diff
		.entrypoint	= &tvConstBias,
		.config		= &physicalEepromMap->sdConfig
	},
	{
		// Linear-steering
		.entrypoint	= &tvLinearBias,
		.config		= &physicalEepromMap->lsConfig
	},
	{
		// Single-tire-fire (left)
		.entrypoint	= &tvConstBias,
		.config		= &STF_L_CONFIG
	},
	{
		// Single-tire-fire (right)
		.entrypoint	= &tvConstBias,
		.config		= &STF_R_CONFIG
	},
	{
		// Bicycle mode
		.entrypoint = &tvBicycleModel,
		.config		= &physicalEepromMap->bicycleConfig
	}
};

/**
 * @brief PID controller responsible for enforcing the global power limit. The y variable represents the vehicle's cumulative
 * power consumption, while the x variable represents the ratio to scale the torque requests by. The set-point is fixed at the
 * power limit, while the output value is clamped from [-1, 0]. This means the controller only has the ability to reduce the
 * requested torque, and this reduction only occurs when the power consumption exceeds the set-point.
 */
static pidController_t powerLimitPid =
{
	.kp			= 0,
	.ki			= 0,
	.kd			= 0,
	.ySetPoint	= 0,
	.ypPrime	= 0,
	.yiPrime	= 0,
	.x			= 0,
	.xp			= 0,
	.xi			= 0,
	.xd			= 0,
};

/// @brief Stores the previous value of the @c powerLimitPid controller's derivative term.
static float powerLimitPidXdPrime = 0.0f;

/// @brief The measurment gain of the @c powerLimitPid controller's low-pass filtering.
static float powerLimitPidA = 0;

// Function Prototypes --------------------------------------------------------------------------------------------------------

/**
 * @brief Calculates the input structure to pass to the selected torque-vectoring algorithm.
 * @param deltaTime The amount of time that has passed since the last call to this function.
 * @return The calculated input structure.
 */
tvInput_t requestCalculateInput (float deltaTime);

/**
 * @brief Executes the selected torque-vectoring algorithm, calculating an amount of torque to request of each inverter.
 * @note This function only calculates the torque request, power limiting and validation should be performed using
 * @c requestApplyPowerLimit and @c requestValidate , respectively.
 * @param input The input structure, as returned by @c requestCalculateInput .
 * @return The output structure containing the torque to request.
 */
tvOutput_t requestCalculateOutput (tvInput_t* input);

/**
 * @brief Applies power-limiting to a torque request, if applicable. See @c powerLimitPid for more details.
 * @param request The request to power-limit.
 * @param deltaTime The amount of time that has passed since the last call to this function.
 * @return True if the request was de-rated, false otherwise.
 */
bool requestApplyPowerLimit (tvOutput_t* request, float deltaTime);

/**
 * @brief Applies regen limiting to a torque amount. De-rating is applied when the motor speed is below a certain value,
 * as negative torque can cause the motors to spin in reverse.
 * @param torque The torque to limit.
 * @param amk The inverter the torque is to be requested of.
 * @return True if the torque was de-rated, false otherwise.
 */
bool torqueApplyRegenLimit (float* torque, amkInverter_t* amk);

/**
 * @brief Checks the validity of a torque request.
 * @param request The request to validate.
 * @param input The input structure that generated said request.
 * @return True if the request is valid, false otherwise.
 */
bool requestValidate (tvOutput_t* request, tvInput_t* input);

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

		// Torque Up / Down buttons
		if (sibGetButtonDownLock (&steeringInputBoard, SIB_TORQUE_UP_INDEX))
			torqueThreadSetDrivingTorqueLimit (drivingTorqueLimit + SIB_TORQUE_STEP);
		if (sibGetButtonDownLock (&steeringInputBoard, SIB_TORQUE_DOWN_INDEX))
			torqueThreadSetDrivingTorqueLimit (drivingTorqueLimit - SIB_TORQUE_STEP);

		// Sample the sensor inputs.
		peripheralsSample (timePrevious, timeCurrent);

		// Calculate the torque request and apply power limiting.
		bool resetRequest = physicalEepromMap->amkAutoResetRequest;
		tvInput_t input = requestCalculateInput (TORQUE_THREAD_PERIOD_S);

		torqueRequest = requestCalculateOutput (&input);
		bool derating = requestApplyPowerLimit (&torqueRequest, TORQUE_THREAD_PERIOD_S);
		bool plausible = requestValidate (&torqueRequest, &input);

		if (vehicleState == VEHICLE_STATE_READY_TO_DRIVE)
		{
			if (plausible)
			{
				// Torque request message.
				derating &= torqueApplyRegenLimit (&torqueRequest.torqueRl, &amkRl);
				amkSendTorqueRequest (&amkRl, torqueRequest.torqueRl, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);

				derating &= torqueApplyRegenLimit (&torqueRequest.torqueRr, &amkRr);
				amkSendTorqueRequest (&amkRr, torqueRequest.torqueRr, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);

				derating &= torqueApplyRegenLimit (&torqueRequest.torqueFl, &amkFl);
				amkSendTorqueRequest (&amkFl, torqueRequest.torqueFl, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);

				derating &= torqueApplyRegenLimit (&torqueRequest.torqueFr, &amkFr);
				amkSendTorqueRequest (&amkFr, torqueRequest.torqueFr, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
			}
			else
			{
				// Send 0 torque request message (keep torque limits, as lowering them in motion will trigger a fault).
				amkSendTorqueRequest (&amkRl, 0, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkRr, 0, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkFl, 0, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
				amkSendTorqueRequest (&amkFr, 0, AMK_DRIVING_TORQUE_MAX, -AMK_REGENERATIVE_TORQUE_MAX, resetRequest, TORQUE_THREAD_CAN_MESSAGE_TIMEOUT);
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
	if (torque > AMK_REGENERATIVE_TORQUE_MAX * AMK_COUNT)
	{
		regenTorqueLimit = AMK_REGENERATIVE_TORQUE_MAX * AMK_COUNT;
		return;
	}

	if (torque < 0)
	{
		regenTorqueLimit = 0;
		return;
	}

	regenTorqueLimit = torque;
}

void torqueThreadSetPowerLimit (float powerLimit)
{
	powerLimitPid.ySetPoint = powerLimit;
}

void torqueThreadSetPowerLimitPid (float kp, float ki, float kd, float a)
{
	powerLimitPid.kp	= kp;
	powerLimitPid.ki	= ki;
	powerLimitPid.kd	= kd;
	powerLimitPidA		= a;
}

tvInput_t requestCalculateInput (float deltaTime)
{
	// Regen input
	float paddleInput0 = sibGetAnalogValueLock (&steeringInputBoard, 0);
	float paddleInput1 = sibGetAnalogValueLock (&steeringInputBoard, 1);
	float regenRequest = paddleInput0 > paddleInput1 ? paddleInput0 : paddleInput1;
	regenRequest *= regenTorqueLimit;

	// Derate based on throttle position (no regen when pedal is depressed)
	regenRequest = lerp2dSaturated (pedals.appsRequest,
		physicalEepromMap->regenDeratingThrottleStart, regenRequest,
		physicalEepromMap->regenDeratingThrottleEnd, 0);

	tvInput_t input =
	{
		.deltaTime			= deltaTime,
		.drivingTorqueLimit	= pedals.appsRequest * drivingTorqueLimit,
		.regenTorqueLimit	= regenRequest,
	};
	return input;
}

tvOutput_t requestCalculateOutput (tvInput_t* input)
{
	// Use the user-specified torque-vectoring algorithm to calculate the torque request.
	return tvAlgorithms [algoritmIndex].entrypoint (input, tvAlgorithms [algoritmIndex].config);
}

bool requestApplyPowerLimit (tvOutput_t* output, float deltaTime)
{
	// Calculate the cumulative power consumption of the inverters.
	float cumulativePower = amksGetCumulativePower (amks, AMK_COUNT);

	// Calculate the torque reduction ratio.
	pidCalculate (&powerLimitPid, cumulativePower, deltaTime);
	pidFilterDerivative (&powerLimitPid, powerLimitPidA, &powerLimitPidXdPrime);
	float torqueReductionRatio = pidApplyAntiWindup (&powerLimitPid, -1.0f, 0.0f) + 1.0f;

	// Scale the torque requests equally by the reduction ratio.
	output->torqueRl *= torqueReductionRatio;
	output->torqueRr *= torqueReductionRatio;
	output->torqueFl *= torqueReductionRatio;
	output->torqueFr *= torqueReductionRatio;

	// If reduction ratio is not 1, derating is occurring.
	return (1.0f - torqueReductionRatio < FLT_EPSILON);
}

bool torqueApplyRegenLimit (float* torque, amkInverter_t* amk)
{
	// Negative torque means regen
	if (*torque < 0)
	{
		// If the speed is below the end derating speed, clamp to 0.
		if (amk->actualSpeed < physicalEepromMap->regenDeratingSpeedEnd)
		{
			*torque = 0;
			return true;
		}
		// If the speed is between the start and end derating speeds, lerp between them.
		else if (amk->actualSpeed < physicalEepromMap->regenDeratingSpeedStart)
		{
			*torque = lerp2d (amk->actualSpeed,
				physicalEepromMap->regenDeratingSpeedEnd, 0,
				physicalEepromMap->regenDeratingSpeedStart, *torque);
			return true;
		}
	}

	return false;
}

bool requestValidate (tvOutput_t* output, tvInput_t* input)
{
	// Check the algorithm's output validity.
	bool valid = output->valid;

	// Check the pedal plausibility.
	valid &= pedals.plausible;

	// Calculate the cumulative driving and regen torques. These are calculated separately, as regen shouldn't allow more than
	// the max driving torque to be requested, and vice versa.
	float drivingTorque = 0.0f;
	float regenTorque = 0.0f;

	if (output->torqueRl >= 0.0f)
		drivingTorque += output->torqueRl;
	else
	 	regenTorque -= output->torqueRl;

	if (output->torqueRr >= 0.0f)
		drivingTorque += output->torqueRr;
	else
	 	regenTorque -= output->torqueRr;;

	if (output->torqueFl >= 0.0f)
		drivingTorque += output->torqueFl;
	else
		regenTorque -= output->torqueFl;

	if (output->torqueFr >= 0.0f)
		drivingTorque += output->torqueFr;
	else
	 	regenTorque -= output->torqueFr;

	// Validate the cumulative torque limits are not exceeded.
	valid &= drivingTorque <= input->drivingTorqueLimit * (1 + CUMULATIVE_TORQUE_TOLERANCE);
	valid &= regenTorque >= -input->regenTorqueLimit * (1 + CUMULATIVE_TORQUE_TOLERANCE);

	return valid;
}