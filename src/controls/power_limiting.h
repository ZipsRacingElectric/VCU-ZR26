// Power Limiting -------------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2026.05.26
//
// Description: PID controller responsible for enforcing the global power limit. The y variable represents the vehicle's
//   cumulative power consumption, while the x variable represents the ratio to scale the torque requests by. The set-point is
//   fixed at the power limit, while the output value is clamped from [-1, 0]. This means the controller only has the ability
//   to reduce the per-motor torque limit, and this reduction only occurs when the power consumption exceeds the set-point.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "controls/pid_controller.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	pidController_t pid;
	float ka;
	float xdPrime;
} powerLimiter_t;

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes a vehicle power limiter.
 * @param limiter The limiter to init.
 * @param kp The proportional gain to use.
 * @param ki The integral gain to use.
 * @param kd The derivative gain to use.
 * @param ka The measurement gain to use.
 * @param powerLimit The vehicle's power limit, in Watts.
 */
void powerLimiterInit (powerLimiter_t* limiter, float kp, float ki, float kd, float ka, float powerLimit);

/**
 * @brief Calculates the ratio to reduce the torque request by in order to satisfy the vehicle's power limit.
 * @param limiter The limiter to use.
 * @param power The measured power consumption, in Watts.
 * @param deltaTime The amount of time that has passed since the last call to this function.
 * @return A scalar the torque request should be scaled by.
 */
float powerLimiterCalculateTorqueLimit (powerLimiter_t* limiter, float power, float deltaTime);