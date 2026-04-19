#ifndef PEDALS_H
#define PEDALS_H

// Pedals ---------------------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.10.04
//
// Description: Objects and functions related to the pedals of the vehicle.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "peripherals/interface/analog_sensor.h"

// ChibiOS
#include "ch.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{
	/// @brief The absolute minimum plausible sample, any lower indicates implausibility.
	uint16_t absoluteMin;
	/// @brief The starting sample at which the sensor request begins increasing.
	uint16_t requestMin;
	/// @brief The ending sample at which the sensor request stops increasing.
	uint16_t requestMax;
	/// @brief The absolute maximum plausible sample, any higher indicates implausibility.
	uint16_t absoluteMax;
} pedalSensorConfig_t;

/**
 * @brief Represents an sensor measuring the position of a pedal.
 */
typedef struct
{
	ANALOG_SENSOR_FIELDS;
	pedalSensorConfig_t*	config;
	uint16_t				sample;
	float					value;
} pedalSensor_t;

typedef struct
{
	/// @brief The configuration of the APPS-1 sensor.
	pedalSensorConfig_t apps1Config;
	/// @brief The configuration of the APPS-2 sensor.
	pedalSensorConfig_t apps2Config;
	/// @brief The configuration of the BSE-F sensor.
	pedalSensorConfig_t bseFConfig;
	/// @brief The configuration of the BSE-R sensor.
	pedalSensorConfig_t bseRConfig;
} pedalsConfig_t;

/**
 * @brief Structure representing the pedals of the vehicle.
 */
typedef struct
{
	/// @brief The 1st accelerator pedal position sensor (APPS).
	pedalSensor_t apps1;
	/// @brief The 2nd accelerator pedal position sensor (APPS).
	pedalSensor_t apps2;
	/// @brief The front brake system encoder sensor (BSE).
	pedalSensor_t bseF;
	/// @brief The rear brake system encoder sensor (BSE).
	pedalSensor_t bseR;

	/// @brief Indicates the complete validity of the pedal requests (includes 100ms timeout).
	bool plausible;
	/// @brief Indicates the instantaneous plausibility of the pedal requests.
	bool plausibleInst;
	/// @brief The deadline for the pedals' plausibility to expire.
	systime_t plausibilityDeadline;
	/// @brief Indicates the instantaneous APPS 10% plausibility.
	bool apps10PercentPlausible;
	/// @brief Indicates the instantaneous APPS 25% / 5% plausibility.
	bool apps25_5Plausible;

	/// @brief Indicates whether the request to accelerate is being made.
	bool accelerating;
	/// @brief Indicates whether the request to brake is being made.
	bool braking;

	/// @brief The requested throttle value.
	float appsRequest;
	/// @brief The requested braking value.
	float bseRequest;

} pedals_t;

// Functions ------------------------------------------------------------------------------------------------------------------

bool pedalSensorInit (pedalSensor_t* sensor, pedalSensorConfig_t* config);

bool pedalsInit (pedals_t* pedals, pedalsConfig_t* config);

/**
 * @brief Updates the state of a pedals peripheral based on the last read samples of the APPS and BSE sensors.
 * @param pedals The peripheral to update.
 * @param timePrevious The system time this function was last called.
 * @param timeCurrent The current system time.
 */
void pedalsUpdate (pedals_t* pedals, systime_t timePrevious, systime_t timeCurrent);

#endif // PEDALS_H