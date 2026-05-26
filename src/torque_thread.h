#ifndef TORQUE_THREAD_H
#define TORQUE_THREAD_H

// Torque Thread --------------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.10.12
//
// Description: Thread implementing the torque control loop. The torque thread has a internal array of torque-vectoring
//   algorithms, allowing the user to select a specific implementation to use. The overall power level of algorithm can be
//   set using the driving and regen torque limits.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "controls/torque_vectoring.h"

// ChibiOS
#include "ch.h"

// Global Data ----------------------------------------------------------------------------------------------------------------

/// @brief The last calculated torque request. Only used for debugging, do not modify.
extern tvOutput_t nonDeratedOutput;

/// @brief The cumulative amount of driving (positive) torque that can be distributed across all motors.
extern float drivingTorqueLimit;

/// @brief The cumulative amount of regenerative (negative) torque that can be distributed across all motors.
extern float regenTorqueLimit;

/// @brief The front-to-rear bias for distributing driving torque. Only used by certain torque vectoring algorithms.
extern float drivingFrontRearBias;

/// @brief The front-to-rear bias for distributing regen torque. Only used by certain torque vectoring algorithms.
extern float regenFrontRearBias;

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief Starts the torque control thread.
 * @param priority The priority to start the thread at.
 */
void torqueThreadStart (tprio_t priority);

/**
 * @brief Selects a torque-vectoring algorithm to use.
 * @param index The index of the algorithm to use.
 */
void torqueThreadSelectAlgorithm (uint8_t index);

/**
 * @brief Sets the cumulative driving (positive) torque limit.
 * @param torque The limit to set, in Nm.
 */
void torqueThreadSetDrivingTorqueLimit (float torque);

/**
 * @brief Sets the cumulative regenerative (negative) torque limit.
 * @param torque The limit to set, in Nm. Should be negative, but positive values will be negated and accepted.
 */
void torqueThreadSetRegenTorqueLimit (float torque);

/**
 * @brief Configures the PID coefficients of the power limiter.
 */
void torqueThreadUpdatePowerLimiter (void);

/// @brief Sets the driving front-to-rear bias.
void torqueThreadSetDrivingFrBias (float bias);

/// @brief Sets the regen front-to-rear bias.
void torqueThreadSetRegenFrBias (float bias);

#endif // TORQUE_THREAD_H