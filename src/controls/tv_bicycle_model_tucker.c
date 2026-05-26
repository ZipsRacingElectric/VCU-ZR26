// Header
#include "tv_bicycle_model_tucker.h"

// Includes
#include "peripherals.h"
#include "can.h"
#include "can/transmit.h"
#include "controls/lerp.h"
#include "controls/vehicle_dynamics.h"

// Constants ------------------------------------------------------------------------------------------------------------------

/// @brief The maximum steering angle defined by the lookup table, in degrees.
#define STEERING_ANGLE_MAX 110

/// @brief The width of the loopup table along the steering angle axis. Note this axis is mirrored.
#define STEERING_ANGLE_WIDTH 4

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

static float calculateYawRateIdeal (float steeringAngle, float vehicleSpeed)
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

tvOutput_t tvBicycleModelTucker (const tvInput_t* input, const void* configPointer, void* statePointer)
{
	// Cast the config and state pointers
	const tvBicycleModelTuckerConfig_t* config = configPointer;
	tvBicycleModelTuckerState_t* state = statePointer;

	bool valid = true;

	// Update PID coefficients
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
	float wheelFlAngle = steeringAngleToFlWheelAngle (steeringAngle, physicalEepromMap->steeringRatio);
	float wheelFrAngle = steeringAngleToFrWheelAngle (steeringAngle, physicalEepromMap->steeringRatio);

	// TODO(Barach): These are quite slow, validate the control frequency is being achieved.
	volatile float yawTransferRl = motorRlYawMomentTransfer (physicalEepromMap->trackWidthRear, physicalEepromMap->gearRatio,
		physicalEepromMap->wheelRadius);
	volatile float yawTransferRr = motorRrYawMomentTransfer (physicalEepromMap->trackWidthRear, physicalEepromMap->gearRatio,
		physicalEepromMap->wheelRadius);
	volatile float yawTransferFl = motorFlYawMomentTransfer (physicalEepromMap->wheelBase, physicalEepromMap->weightFrontRearBias,
		physicalEepromMap->trackWidthFront, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius, wheelFlAngle);
	volatile float yawTransferFr = motorFrYawMomentTransfer (physicalEepromMap->wheelBase, physicalEepromMap->weightFrontRearBias,
		physicalEepromMap->trackWidthFront, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius, wheelFrAngle);

	volatile float torqueRr =      input->drivingFrBias  * 0.5f * input->drivingTorqueLimit;
	volatile float torqueFl = (1 - input->drivingFrBias) * 0.5f * input->drivingTorqueLimit;

	volatile float yawMomentMin =
		  torqueRr * yawTransferRl
		+ torqueFl * yawTransferFr
		+ (-AMK_REGENERATIVE_TORQUE_MAX) * yawTransferRl
		+ (AMK_DRIVING_TORQUE_MAX) * yawTransferFr;

	volatile float yawMomentMax =
		  torqueRr * yawTransferRl
		+ torqueFl * yawTransferFr
		+ AMK_DRIVING_TORQUE_MAX * yawTransferRl
		+ (-AMK_REGENERATIVE_TORQUE_MAX) * yawTransferFr;

	// Compute ideal yaw rate from steering angle and vehicle speed.
	volatile float yawRateIdeal = calculateYawRateIdeal (steeringAngle, vehicleSpeed);

	// Compute the yaw acceleration based on a PID controller
	state->pid.ySetPoint = yawRateIdeal;
	pidCalculate (&state->pid, yawRateActual, input->deltaTime);
	pidFilterDerivative (&state->pid, config->ka, &state->xdPrime);
	volatile float yawAcceleration = pidApplyAntiWindup (&state->pid,
		yawMomentMin / physicalEepromMap->yawMomentOfInertia,
		yawMomentMax / physicalEepromMap->yawMomentOfInertia);

	volatile float yawMoment = yawAcceleration * physicalEepromMap->yawMomentOfInertia;

	volatile float yawMomentRequired = yawMoment - torqueRr * yawTransferRr - torqueFl * yawTransferFl;

	// Transmit the yaw-rate message
	transmitYawRateMessage (&CAND1, yawRateActual, yawRateIdeal, yawMoment, TIME_MS2I (10));

	// Compute the torques that satisfy both the torque limit and required yaw moment.
	float torqueRl =  (yawMomentRequired - 0.5f * input->drivingTorqueLimit * yawTransferFr) / (yawTransferRl - yawTransferFr);
	float torqueFr = -(yawMomentRequired - 0.5f * input->drivingTorqueLimit * yawTransferRl) / (yawTransferRl - yawTransferFr);

	return (tvOutput_t)
	{
		.valid = valid,
		.torqueRl = torqueRl,
		.torqueRr = torqueRr,
		.torqueFl = torqueFl,
		.torqueFr = torqueFr,
	};
}