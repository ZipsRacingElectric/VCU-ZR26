// Header
#include "power_limiting.h"

void powerLimiterInit (powerLimiter_t* limiter, float kp, float ki, float kd, float ka, float powerLimit)
{
	*limiter = (powerLimiter_t)
	{
		.pid =
		{
			.kp		= kp,
			.ki		= ki,
			.kd		= kd,
			.x		= powerLimit,
			.xp		= 0,
			.xi		= 0,
			.xd		= 0
		},
		.ka			= ka,
		.xdPrime	= 0
	};
}

float powerLimiterCalculateTorqueLimit (powerLimiter_t* limiter, float power, float deltaTime)
{
	// Calculates a ratio to reduce the torque request by. The PID output ranges from [-1, 0], which is offset to yield [0, 1].
	pidCalculate (&limiter->pid, power, deltaTime);
	pidFilterDerivative (&limiter->pid, limiter->ka, &limiter->xdPrime);
	return pidApplyAntiWindup (&limiter->pid, -1.0f, 0.0f) + 1.0f;
}