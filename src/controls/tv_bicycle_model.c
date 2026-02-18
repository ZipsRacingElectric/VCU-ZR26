// Header
#include "tv_bicycle_model.h"

// Includes
#include "peripherals.h"
#include "can.h"

// C Standard Library
#define _USE_MATH_DEFINES
#include <math.h>

/**
 * @brief Simple bicycle model: r = vx / L * tan (delta)
 * @param steerAngle Angle of the vehicle's front wheels (rad)
 * @param wheelBase The distance between the front and rear wheels, in meters.
 * @param speed The reference speed of the vehicle, in meters per second.
 * @return The expected yaw rate, in radians / second.
 */
static float calculateExpectedYawRate (float steerAngle, float wheelBase, float speed)
{
	return (speed / wheelBase) * tanf (steerAngle);
}

tvOutput_t tvBicycleModel (const tvInput_t* input, const void* configPointer)
{
	const tvBicycleModelConfig_t* config = configPointer;

	// Get IMU-based measurements

	canNodeLock ((canNode_t*) &gps);

	if (gps.state != CAN_NODE_VALID)
	{
		canNodeUnlock ((canNode_t*) &gps);
		return (tvOutput_t) { .valid = false };
	}

	// deg/s => rad/s
	// TODO(Barach): Replace with M_PI
	float measuredYawRate = gps.yAngleRate * 3.141592653589793238f / 180.0f;

	// km/h => m/s
	float measuredSpeed = gps.speed * 1000.0f / 3600.0f;

	canNodeUnlock ((canNode_t*) &gps);

	if (sas.state != ANALOG_SENSOR_VALID)
		return (tvOutput_t) { .valid = false };
	// TODO(Barach): Constant conversion for steering angle to steer angle.
	float measuredSteerAngle = sas.value * 1 / 11.0f;

	// Compute expected yaw rate from steering
	float targetYawRate = calculateExpectedYawRate (measuredSteerAngle, config->wheelBase, measuredSpeed);

	// Compute yaw correction
	float yawCorrection = (measuredYawRate - targetYawRate) * config->gain;

	// Calculate the amount of torque to distribute to each wheel
	float biasF = 1 - config->biasFr;
	float biasR = config->biasFr;
	float biasRl = 0;
	float biasRr = 0;
	float biasFl = 0;
	float biasFr = 0;
	if (yawCorrection > 0)
	{
		// Oversteer condition

		if (targetYawRate > 0)
		{
			// Right-hand turn
			biasRl = pedals.appsRequest - yawCorrection;
			biasRr = pedals.appsRequest;
			biasFl = pedals.appsRequest;
			biasFr = pedals.appsRequest + yawCorrection;
		}
		else
		{
			// Left-hand turn
			biasRl = pedals.appsRequest;
			biasRr = pedals.appsRequest - yawCorrection;
			biasFl = pedals.appsRequest + yawCorrection;
			biasFr = pedals.appsRequest;
		}
	}
	else
	{
		// Understeer condition
		// Note: Yaw correction is less than 0, so signs are inverted here.

		if (targetYawRate > 0)
		{
			// Right-hand turn
			biasRl = pedals.appsRequest;
			biasRr = pedals.appsRequest - yawCorrection;
			biasFl = pedals.appsRequest + yawCorrection;
			biasFr = pedals.appsRequest;
		}
		else
		{
			// Left-hand turn
			biasRl = pedals.appsRequest - yawCorrection;
			biasRr = pedals.appsRequest;
			biasFl = pedals.appsRequest;
			biasFr = pedals.appsRequest + yawCorrection;
		}
	}

	return (tvOutput_t)
	{
		.valid = true,
		.torqueRl = biasRl * biasR * 0.5f * input->drivingTorqueLimit,
		.torqueRr = biasRr * biasR * 0.5f * input->drivingTorqueLimit,
		.torqueFl = biasFl * biasF * 0.5f * input->drivingTorqueLimit,
		.torqueFr = biasFr * biasF * 0.5f * input->drivingTorqueLimit
	};
}