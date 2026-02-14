#ifndef STEERING_ANGLE_H
#define STEERING_ANGLE_H

// Steering Angle Sensor (SAS) ------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.10.24
//
// Description: Object and functions related to the steering angle sensor.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "peripherals/interface/analog_sensor.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	/// @brief The sampled value which should read as zero.
	uint16_t sampleZero;
	/// @brief The sample correlating to the negative-most position.
	uint16_t sampleNegative;
	/// @brief The sample correlating to the positive-most position.
	uint16_t samplePositive;
	/// @brief The angle of the negative-most position, in radians.
	float angleNegative;
	/// @brief The angle of the positive-most position, in radians.
	float anglePositive;
	/// @brief The range of the deadzone centered at the zero position.
	float angleDeadzone;
} sasConfig_t;

typedef struct
{
	ANALOG_SENSOR_FIELDS;
	sasConfig_t*	config;
	uint16_t		sample;
	float			value;
} sas_t;

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the sensor using the specified configuration.
 * @param sas The sensor to initialize.
 * @param config The configuration to use.
 * @return True if successful, false otherwise.
 */
bool sasInit (sas_t* sas, sasConfig_t* config);

#endif // STEERING_ANGLE_H