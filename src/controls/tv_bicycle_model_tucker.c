// Header
#include "tv_bicycle_model_tucker.h"

// Includes
#include "peripherals.h"
#include "can/transmit.h"
#include "controls/lerp.h"
#include "controls/vehicle_dynamics.h"

// Constants ------------------------------------------------------------------------------------------------------------------

/// @brief The maximum steering angle defined by the lookup table, in degrees.
#define STEERING_ANGLE_MAX 110

/// @brief The width of the loopup table along the steering angle axis. Note this axis is mirrored.
#define STEERING_ANGLE_WIDTH 4

// TODO(Barach): Update this, CAN node, and DBC file to km/h (probably dash too).
/// @brief The maximum vehicle speed defined by the loopup table, in mph.
#define VEHICLE_SPEED_MAX 1

/// @brief The width of the loopup table along the vehicle speed axis. This axis is not mirrored.
#define VEHICLE_SPEED_WIDTH 5

/// @brief The loopup table of ideal yaw rates, in degrees per second.
/// @note This is generated from MATLAB. TODO(Barach): Link here.
static const float YAW_LOOKUP_TABLE [VEHICLE_SPEED_WIDTH][STEERING_ANGLE_WIDTH] =
{
	{ 0,  0,  0,  0},
	{ 0,  1,  2,  3},
	{ 0,  2,  4,  6},
	{ 0,  4,  8, 12},
	{ 0,  8, 16, 24},
};

// Functions ------------------------------------------------------------------------------------------------------------------

static inline void calculateYawTransfers (float* yawTransferRearOuter, float* yawTransferRearInner,
	float* yawTransferFrontOuter, float* yawTransferFrontInner, float steeringAngle, bool rightHandSteering)
{
	// Calculate the FL and FR steering wheel angles.
	float wheelFlAngle = steeringAngleToFlWheelAngle (steeringAngle, physicalEepromMap->steeringRatio);
	float wheelFrAngle = steeringAngleToFrWheelAngle (steeringAngle, physicalEepromMap->steeringRatio);

	// Calculate each wheel's motor-torque-to-yaw-moment transfer scalar.
	float yawTransferRl = motorRlYawMomentTransfer (physicalEepromMap->trackWidthRear, physicalEepromMap->gearRatio,
			physicalEepromMap->wheelRadius);
	float yawTransferRr = motorRrYawMomentTransfer (physicalEepromMap->trackWidthRear, physicalEepromMap->gearRatio,
			physicalEepromMap->wheelRadius);
	float yawTransferFl = motorFlYawMomentTransfer (physicalEepromMap->wheelBase, physicalEepromMap->weightFrontRearBias,
			physicalEepromMap->trackWidthFront, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius, wheelFlAngle);
	float yawTransferFr = motorFrYawMomentTransfer (physicalEepromMap->wheelBase, physicalEepromMap->weightFrontRearBias,
			physicalEepromMap->trackWidthFront, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius, wheelFrAngle);

	// Map each wheel's scalar to the appropriate output
	*yawTransferRearOuter  = rightHandSteering ? yawTransferRl : yawTransferRr;
	*yawTransferRearInner  = rightHandSteering ? yawTransferRr : yawTransferRl;
	*yawTransferFrontOuter = rightHandSteering ? yawTransferFl : yawTransferFr;
	*yawTransferFrontInner = rightHandSteering ? yawTransferFr : yawTransferFl;
}

static inline float calculateYawRateIdeal (float steeringAngle, float vehicleSpeed)
{
	// If the steering angle is negative, mirror the table
	bool mirror = false;
	if (steeringAngle < 0)
	{
		steeringAngle = -steeringAngle;
		mirror = true;
	}

	// Get the ideal yaw rate from the lookup table (with bilinear interpolation)
	float yawRateIdeal = bilinearLookupTable (steeringAngle, vehicleSpeed, 0, STEERING_ANGLE_MAX, 0, VEHICLE_SPEED_MAX,
		(const float*) YAW_LOOKUP_TABLE, STEERING_ANGLE_WIDTH, VEHICLE_SPEED_WIDTH);

	// If we are mirroring the table, negate the yaw rate.
	return mirror ? -yawRateIdeal : yawRateIdeal;
}

// Entrypoint -----------------------------------------------------------------------------------------------------------------

tvOutput_t tvBicycleModelTucker (const tvInput_t* input, const void* configPointer, void* statePointer)
{
	// TODO(Barach): This only does right-hand turning, need to figure out how to mirror.

	// Cast the config and state pointers
	const tvBicycleModelTuckerConfig_t* config = configPointer;
	tvBicycleModelTuckerState_t* state = statePointer;

	bool valid = true;

	// Update PID coefficients (in case EEPROM is updated)
	state->pid.kp = config->kp;
	state->pid.ki = config->ki;
	state->pid.kd = config->kd;

	// Get IMU-based measurements

	float yawRateActual = (pedals.bseF.value - 0.5f) * 48.0f;
	float vehicleSpeed = pedals.bseR.value;

	// canNodeLock ((canNode_t*) &ecumaster);

	// valid &= ecumaster.state == CAN_NODE_VALID;
	// float yawRateActual = ecumaster.zAngleRate;
	// float vehicleSpeed = ecumaster.speed;

	// canNodeUnlock ((canNode_t*) &ecumaster);

	// Get SAS-based measurements
	valid &= sas.state == ANALOG_SENSOR_VALID;
	float steeringAngle = sas.value;

	// Determine whether the vehicle is steering to the left or right. If not steering (due to the deadzone), use the last
	// direction the vehicle was steering in.
	bool rightHandSteering;
	if (steeringAngle > 0.01f)
		rightHandSteering = true;
	else if (steeringAngle < -0.01f)
		rightHandSteering = false;
	else
		rightHandSteering = state->rightHandSteering;

	state->rightHandSteering = rightHandSteering;

	volatile float torqueLimitRearOuter;
	volatile float torqueLimitFrontInner;
	volatile float regenLimitRearOuter;
	volatile float regenLimitFrontInner;

	// Calculate the motor-torque-to-yaw-moment transfer coefficients.
	// - TODO(Barach): This is quite slow, validate the control frequency is being achieved.
	float yawTransferRearOuter, yawTransferRearInner, yawTransferFrontOuter, yawTransferFrontInner;
	calculateYawTransfers (&yawTransferRearOuter, &yawTransferRearInner,
		&yawTransferFrontOuter, &yawTransferFrontInner, steeringAngle, rightHandSteering);

	torqueLimitRearOuter  = rightHandSteering ? input->drivingTorqueLimitRl : input->drivingTorqueLimitRr;
	torqueLimitFrontInner = rightHandSteering ? input->drivingTorqueLimitFr : input->drivingTorqueLimitFl;
	regenLimitRearOuter   = rightHandSteering ? input->regenTorqueLimitRl   : input->regenTorqueLimitRr;
	regenLimitFrontInner  = rightHandSteering ? input->regenTorqueLimitFr   : input->regenTorqueLimitFl;

	// Calculate the rear-inner and front-outer wheel torque.
	// - These two are not used to apply yaw moment.
	volatile float torqueRearInner  =      input->drivingFrBias  * 0.5f * input->torqueRequest;
	volatile float torqueFrontOuter = (1 - input->drivingFrBias) * 0.5f * input->torqueRequest;

	// TODO(Barach): This is not correct.

	// // Minimum yaw acceleration is max regen on the rear-outer and max torque on the front-inner.
	// volatile float yawAccelerationMin =
	// 	 (torqueRearInner       * yawTransferRearInner
	// 	+ torqueFrontOuter      * yawTransferFrontOuter
	// 	+ regenLimitRearOuter   * yawTransferRearOuter
	// 	+ torqueLimitFrontInner * yawTransferFrontInner) / physicalEepromMap->yawMomentOfInertia;

	// // Maximum yaw acceleration is max torque on the rear-outer and max regen on the front-inner.
	// volatile float yawAccelerationMax =
	// 	 (torqueRearInner       * yawTransferRearInner
	// 	+ torqueFrontOuter      * yawTransferFrontOuter
	// 	+ torqueLimitRearOuter  * yawTransferRearOuter
	// 	+ regenLimitFrontInner  * yawTransferFrontInner) / physicalEepromMap->yawMomentOfInertia;

	// Calculate the ideal yaw rate from steering angle and vehicle speed.
	volatile float yawRateIdeal = calculateYawRateIdeal (steeringAngle, vehicleSpeed);

	// Calculate the target yaw acceleration based on a PID controller
	state->pid.ySetPoint = yawRateIdeal;
	pidCalculate (&state->pid, yawRateActual, input->deltaTime);
	pidFilterDerivative (&state->pid, config->ka, &state->xdPrime);
	volatile float yawAcceleration = pidApplyAntiWindup (&state->pid, -3000, 3000);

	// Calculate the target yaw moment and required contribution of the remaining wheels.
	volatile float yawMoment = yawAcceleration * physicalEepromMap->yawMomentOfInertia;
	volatile float yawMomentRequired = yawMoment
		- torqueRearInner  * yawTransferRearInner
		- torqueFrontOuter * yawTransferFrontOuter;

	// Transmit the yaw-rate message
	transmitYawRateMessage (&CAND1, yawRateActual, yawRateIdeal, yawMoment, TIME_MS2I (10));

	// Calculate the torques that satisfy both the torque limit and required yaw moment.
	float denominator = (yawTransferRearOuter - yawTransferFrontInner);
	float torqueRearOuter  =  (yawMomentRequired - 0.5f * input->torqueRequest * yawTransferFrontInner) / denominator;
	float torqueFrontInner = -(yawMomentRequired - 0.5f * input->torqueRequest * yawTransferRearOuter)  / denominator;

	// Saturate the motor requests

	if (torqueRearOuter > torqueLimitRearOuter)
	{
		torqueRearOuter  = torqueLimitRearOuter;
		torqueFrontInner = 0.5f * input->torqueRequest - torqueLimitRearOuter;
	}

	if (torqueFrontInner > torqueLimitFrontInner)
	{
		torqueFrontInner = torqueLimitFrontInner;
		torqueRearOuter = 0.5f * input->torqueRequest - torqueLimitFrontInner;
	}

	if (torqueRearOuter < regenLimitRearOuter)
	{
		torqueRearOuter = regenLimitRearOuter;
		torqueFrontInner = 0.5f * input->torqueRequest - regenLimitRearOuter;
	}

	if (torqueFrontInner < regenLimitFrontInner)
	{
		torqueFrontInner = regenLimitFrontInner;
		torqueRearOuter = 0.5f * input->torqueRequest - regenLimitFrontInner;
	}

	// Map the motor torques to the output based on the steering direction.
	return (tvOutput_t)
	{
		.valid = valid,
		.torqueRl = rightHandSteering ? torqueRearOuter  : torqueRearInner,
		.torqueRr = rightHandSteering ? torqueRearInner  : torqueRearOuter,
		.torqueFl = rightHandSteering ? torqueFrontOuter : torqueFrontInner,
		.torqueFr = rightHandSteering ? torqueFrontInner : torqueFrontOuter,
	};
}