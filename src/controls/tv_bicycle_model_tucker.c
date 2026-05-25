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

	// Compute ideal yaw rate from steering angle and vehicle speed.
	float yawRateIdeal = calculateYawRateIdeal (steeringAngle, vehicleSpeed);

	// Compute the yaw acceleration based on a PID controller
	state->pid.ySetPoint = yawRateIdeal;
	pidCalculate (&state->pid, yawRateActual, input->deltaTime);
	float yawAcceleration = pidFilterDerivative (&state->pid, config->ka, &state->xdPrime);
	// TODO(Barach): do we need antiwindup? saturation is likely not constant.
	// TODO(Barach): Yes, but it is complicated...
	// volatile float yawAcceleration = pidApplyAntiWindup (&state->pid, 0, AMK_DRIVING_TORQUE_MAX);
	volatile float yawMoment = yawAcceleration * physicalEepromMap->yawMomentOfInertia;

	// Transmit the yaw-rate message
	transmitYawRateMessage (&CAND1, yawRateActual, yawRateIdeal, yawMoment, TIME_MS2I (10));

	volatile float yawMomentRear  = yawMoment * config->frontRearMomentBias;
	volatile float yawMomentFront = yawMoment * (1 - config->frontRearMomentBias);

	volatile float torqueLimitRear  = input->drivingTorqueLimit * input->drivingFrBias;
	volatile float torqueLimitFront = input->drivingTorqueLimit * (1 - input->drivingFrBias);

	volatile float yawTranfserRl = motorRlYawMomentTransfer (physicalEepromMap->trackWidthRear, physicalEepromMap->gearRatio,
		physicalEepromMap->wheelRadius);
	volatile float yawTranfserRr = motorRrYawMomentTransfer (physicalEepromMap->trackWidthRear, physicalEepromMap->gearRatio,
		physicalEepromMap->wheelRadius);
	volatile float yawTranfserFl = motorFlYawMomentTransfer (physicalEepromMap->wheelBase, physicalEepromMap->frontRearWeightBias,
		physicalEepromMap->trackWidthFront, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius, wheelFlAngle);
	volatile float yawTranfserFr = motorFrYawMomentTransfer (physicalEepromMap->wheelBase, physicalEepromMap->frontRearWeightBias,
		physicalEepromMap->trackWidthFront, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius, wheelFrAngle);

	volatile float yawTransferDeltaRear  = yawTranfserRl - yawTranfserRr;
	volatile float yawTransferDeltaFront = yawTranfserFl - yawTranfserFr;

	// Compute the compensated output
	return (tvOutput_t)
	{
		.valid = valid,
		// Note this is not a mistake, the right-hand yaw transfer is used to calculate the left-hand torque and vice-versa.
		.torqueRl = ( yawMomentRear  - yawTranfserRr * torqueLimitRear)  / yawTransferDeltaRear,
		.torqueRr = (-yawMomentRear  + yawTranfserRl * torqueLimitRear)  / yawTransferDeltaRear,
		.torqueFl = ( yawMomentFront - yawTranfserFr * torqueLimitFront) / yawTransferDeltaFront,
		.torqueFr = (-yawMomentFront + yawTranfserFl * torqueLimitFront) / yawTransferDeltaFront,
	};
}