// Header
#include "power_limiting.h"

// Includes
#include "controls/vehicle_dynamics.h"

#include "can/transmit.h"

void powerLimitDistribute (float electricalPowerLimit, float efficiencyEstimate, float drivingFrBias,
	float* mechanicalPowerLimitRl, float* mechanicalPowerLimitRr, float* mechanicalPowerLimitFl, float* mechanicalPowerLimitFr)
{
	float mechanicalPowerLimit = electricalPowerLimit * efficiencyEstimate;

	*mechanicalPowerLimitRl = mechanicalPowerLimit * 0.5f * drivingFrBias;
	*mechanicalPowerLimitRr = *mechanicalPowerLimitRl;
	*mechanicalPowerLimitFl = mechanicalPowerLimit * 0.5f * (1.0f - drivingFrBias);
	*mechanicalPowerLimitFr = *mechanicalPowerLimitFl;
}

void powerLimiterInit (powerLimiter_t* limiter, float kp, float ki, float kd, float ka)
{
	*limiter = (powerLimiter_t)
	{
		.pid =
		{
			.kp			= kp,
			.ki			= ki,
			.kd			= kd,
			.ySetPoint	= 0,
			.xp			= 0,
			.xi			= 0,
			.xd			= 0
		},
		.ka				= ka,
		.xdPrime		= 0
	};
}

float powerLimiterDerateLimit (powerLimiter_t* limiter, amkInverter_t* amk, float mechanicalPowerLimit, float torquePrevious, float deltaTime)
{
	// Get the motor speed, in radians per second.
	canNodeLock ((canNode_t*) amk);
	// float motorSpeed = RPM_TO_RADIANS_PER_SECOND (amk->actualSpeed);
	float actualPower = amk->actualPower;
	canNodeUnlock ((canNode_t*) amk);

	limiter->pid.ySetPoint = mechanicalPowerLimit;

	// // Calculate the minimum PID output based on the motors limits and the current speed.
	// float mechanicalPowerMin = -AMK_DRIVING_TORQUE_MAX * motorSpeed;
	// if (mechanicalPowerMin > 0)
	// 	mechanicalPowerMin = 0;

	// Calculate the PID output (the amount of mechanical power to reduce the limit by).
	pidCalculate (&limiter->pid, actualPower, deltaTime);
	pidFilterDerivative (&limiter->pid, limiter->ka, &limiter->xdPrime);

	// pidApplyAntiWindup (&limiter->pid, mechanicalPowerMin, 0.0f);
	pidApplyAntiWindup (&limiter->pid, -AMK_DRIVING_TORQUE_MAX, 0.0f);

	// If the power limit was just exceeded, seed the integral term to start derating from the last requested torque.
	// - This is to allow the PID to immediately start derating the requested torque.
	if (actualPower >= mechanicalPowerLimit && limiter->yPrime < mechanicalPowerLimit)
	{
		float torqueReduction = AMK_DRIVING_TORQUE_MAX - torquePrevious;

		// pidSeedIntegral (&limiter->pid, -torqueReduction * motorSpeed);
		pidSeedIntegral (&limiter->pid, -torqueReduction);
	}
	limiter->yPrime = actualPower;

	// // If the motor speed is 0, the PID output must also be 0, so perform no limiting.
	// // - This avoids division by 0.
	// if (motorSpeed <= 0)
	// 	return AMK_DRIVING_TORQUE_MAX;

	// Otherwise, the torque limit is the normal torque limit plus the PID output (converted to a torque).

	// float mechanicalPowerReduction = limiter->pid.x;
	// float torqueReduction = mechanicalPowerReduction / motorSpeed;

	float torqueReduction = limiter->pid.x;

	return AMK_DRIVING_TORQUE_MAX + torqueReduction;
}