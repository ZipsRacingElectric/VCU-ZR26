// Header
#include "pedals.h"

// Includes
#include "controls/lerp.h"

// Constants ------------------------------------------------------------------------------------------------------------------

/// @brief The maximum acceptable delta of the APPS-1 and APPS-2 sensor values.
#define APPS_DELTA_MAX 0.1f

/// @brief The maximum acceptable delta of the BSE-F and BSE-R sensor values.
#define BSE_DELTA_MAX 0.1f

// Function Prototypes --------------------------------------------------------------------------------------------------------

/**
 * @brief Updates the value of a pedal sensor based on a read sample.
 * @param object The pedal sensor to update (must be @c pedalSensor_t ).
 * @param sample The read sample.
 * @param sampleVdd TODO(Barach)
 */
static void callback (void* object, uint16_t sample, uint16_t sampleVdd);

// Functions ------------------------------------------------------------------------------------------------------------------

bool pedalSensorInit (pedalSensor_t* sensor, pedalSensorConfig_t* config)
{
	// Store the configuration
	sensor->config = config;
	sensor->callback = callback;

	// Validate the configuration
	bool configValid =
		config->absoluteMin < config->requestMin &&
		config->requestMin < config->requestMax &&
		config->requestMax < config->absoluteMax;

	sensor->state = configValid ? ANALOG_SENSOR_SAMPLE_INVALID : ANALOG_SENSOR_CONFIG_INVALID;

	// Set values to their defaults
	sensor->value = 0.0f;
	sensor->sample = 0;

	return sensor->state != ANALOG_SENSOR_CONFIG_INVALID;
}

void callback (void* object, uint16_t sample, uint16_t sampleVdd)
{
	// TODO(Barach): How to mange?
	(void) sampleVdd;

	pedalSensor_t* sensor = (pedalSensor_t*) object;

	// Store the sample.
	sensor->sample = sample;

	// If the config is invalid, don't check anything else.
	if (sensor->state == ANALOG_SENSOR_CONFIG_INVALID)
		return;

	// Check the sample is in the valid range
	if (sample < sensor->config->absoluteMin || sample > sensor->config->absoluteMax)
	{
		sensor->state = ANALOG_SENSOR_SAMPLE_INVALID;
		sensor->value = 0;
		return;
	}

	sensor->state = ANALOG_SENSOR_VALID;

	// Inverse lerp in request range, saturate otherwise.
	if (sample < sensor->config->requestMin)
		sensor->value = 0.0f;
	else if (sample < sensor->config->requestMax)
		sensor->value = inverseLerp (sample, sensor->config->requestMin, sensor->config->requestMax);
	else
	 	sensor->value = 1.0f;
}

bool pedalsInit (pedals_t* pedals, pedalsConfig_t* config)
{
	// Configure and validate the individual sensors
	bool result = pedalSensorInit (&pedals->apps1, &config->apps1Config);
	result &= pedalSensorInit (&pedals->apps2, &config->apps2Config);
	result &= pedalSensorInit (&pedals->bseF, &config->bseFConfig);
	result &= pedalSensorInit (&pedals->bseR, &config->bseRConfig);
	return result;
}

void pedalsUpdate (pedals_t* pedals, systime_t timePrevious, systime_t timeCurrent)
{
	// Calculate the APPS & BSE requests (average of both sensors)
	pedals->appsRequest = (pedals->apps1.value + pedals->apps2.value) / 2.0f;
	pedals->bseRequest = (pedals->bseF.value + pedals->bseR.value) / 2.0f;

	// Get the pedals states
	pedals->accelerating = pedals->appsRequest > 0.0f;
	pedals->braking = pedals->bseRequest > 0.0f;

	// APPS 10% check (FSAE Rules T.4.2.4)
	float appsDelta = pedals->apps1.value - pedals->apps2.value;
	pedals->apps10PercentPlausible = appsDelta <= APPS_DELTA_MAX && appsDelta >= -APPS_DELTA_MAX;

	// APPS 25/5 check (FSAE Rules EV.4.7)
	if (pedals->appsRequest > 0.25 && pedals->braking)
		pedals->apps25_5Plausible = false;
	else if (pedals->appsRequest < 0.05)
		pedals->apps25_5Plausible = true;

	// Instantaneous plausiblity check
	pedals->plausibleInst =
		pedals->apps1.state == ANALOG_SENSOR_VALID &&
		pedals->apps2.state == ANALOG_SENSOR_VALID &&
		pedals->bseF.state == ANALOG_SENSOR_VALID &&
		pedals->bseR.state == ANALOG_SENSOR_VALID &&
		pedals->apps10PercentPlausible &&
		pedals->apps25_5Plausible;

	// 100ms plausibility timeout (FSAE Rules T.4.2.5)
	if (pedals->plausibleInst)
	{
		// If we are plausible now, postpone the deadline and set the plausibility true.
		pedals->plausible = true;
		pedals->plausibilityDeadline = chTimeAddX (timeCurrent, TIME_MS2I (100));
	}
	else
	{
		// If we are implausible, check for the deadline expiry.
		if (chTimeIsInRangeX (pedals->plausibilityDeadline, timePrevious, timeCurrent))
			pedals->plausible = false;
	}
}