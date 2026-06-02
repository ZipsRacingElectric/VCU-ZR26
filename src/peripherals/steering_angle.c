// Header
#include "steering_angle.h"

// Includes
#include "controls/lerp.h"

// Constants ------------------------------------------------------------------------------------------------------------------

#define ZERO_SAMPLE 2048

// Function Prototypes --------------------------------------------------------------------------------------------------------

/**
 * @brief Updates the value of the sensor.
 * @note This function uses a @c void* for the object reference as to make the signature usable by callbacks.
 * @param object The sensor to update (must be a @c sas_t* ).
 * @param sample The read sample.
 * @param sampleVdd TODO(Barach)
 */
static void callback (void* object, uint16_t sample, uint16_t sampleVdd);

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief Converts a raw sample value into an offset value (zero is applied).
 * @param sas The SAS to convert for.
 * @param sample The raw sample to convert.
 * @return The sample after offset.
 */
static uint16_t rawSampleToOffsetSample (sas_t* sas, uint16_t sample)
{
	return ((uint16_t) (sample + ZERO_SAMPLE - sas->config->sampleZero)) % 4096;
}

bool sasInit (sas_t* sas, sasConfig_t* config)
{
	// Store the configuration
	sas->config = config;
	sas->callback = callback;

	// Convert the raw sample min / max values into offset values.
	sas->samplePositive = rawSampleToOffsetSample (sas, sas->config->samplePositive);
	sas->sampleNegative = rawSampleToOffsetSample (sas, sas->config->sampleNegative);

	// Validate the configuration
	if (sas->sampleNegative >= ZERO_SAMPLE || ZERO_SAMPLE >= sas->samplePositive)
		sas->state = ANALOG_SENSOR_CONFIG_INVALID;
	else
		sas->state = ANALOG_SENSOR_SAMPLE_INVALID;

	// Set values to their defaults
	sas->value = 0.0f;

	return sas->state != ANALOG_SENSOR_CONFIG_INVALID;
}

void callback (void* object, uint16_t sample, uint16_t sampleVdd)
{
	// TODO(Barach)
	(void) sampleVdd;

	sas_t* sas = (sas_t*) object;

	// Store the sample without the offset
	sas->sample = sample;

	// Apply the sample offset and 4096 modulus
	sample = rawSampleToOffsetSample (sas, sample);

	// If the config is invalid, don't check anything else.
	if (sas->state == ANALOG_SENSOR_CONFIG_INVALID)
		return;

	// Check the sample is in the valid range
	if (sample < sas->sampleNegative || sample > sas->samplePositive)
	{
		sas->state = ANALOG_SENSOR_SAMPLE_INVALID;
		sas->value = 0;
	}

	sas->state = ANALOG_SENSOR_VALID;

	if (sample > ZERO_SAMPLE)
	{
		// Map zero sample to zero angle, positive sample to positive angle.
		sas->value = lerp2d (sample, ZERO_SAMPLE, 0, sas->samplePositive, sas->config->anglePositive);

		// Deadzone check
		if (sas->value <= sas->config->angleDeadzone / 2.0f)
			sas->value = 0.0f;
	}
	else
	{
		// Map negative sample to negative angle, zero sample to zero angle.
		sas->value = lerp2d (sample, sas->sampleNegative, sas->config->angleNegative, ZERO_SAMPLE, 0);

		// Deadzone check
		if (sas->value >= -sas->config->angleDeadzone / 2.0f)
			sas->value = 0.0f;
	}
}