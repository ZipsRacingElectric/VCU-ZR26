// Power Limiting -------------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2026.05.26
//
// Description: TODO(Barach)

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "controls/pid_controller.h"
#include "can/amk_inverter.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	pidController_t pid;
	float ka;
	float xdPrime;
	float yPrime;
} powerLimiter_t;

// Functions ------------------------------------------------------------------------------------------------------------------

void powerLimitDistribute (float electricalPowerLimit, float efficiencyEstimate, float drivingFrBias,
	float* mechanicalPowerLimitRl, float* mechanicalPowerLimitRr, float* mechanicalPowerLimitFl, float* mechanicalPowerLimitFr);

void powerLimiterInit (powerLimiter_t* limiter, float kp, float ki, float kd, float ka);

float powerLimiterDerateLimit (powerLimiter_t* limiter, amkInverter_t* amk, float mechanicalPowerLimit, float torquePrevious, float deltaTime);