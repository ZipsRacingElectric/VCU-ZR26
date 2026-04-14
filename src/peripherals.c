// Header
#include "peripherals.h"

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "torque_thread.h"
#include "controls/lerp.h"

// Global Peripherals ---------------------------------------------------------------------------------------------------------

// Public
stmAdc_t		adc;
mc24lc32_t		physicalEeprom;
virtualEeprom_t virtualEeprom;
linearSensor_t	glvBattery;
pedals_t		pedals;
as5600_t		sasAdc;
sas_t			sas;
bool			bspdFault;

// Private
eeprom_t		readonlyWriteonlyEeprom;

// Configuration --------------------------------------------------------------------------------------------------------------

/// @brief Configuration for the I2C1 bus.
static const I2CConfig I2C1_CONFIG =
{
	.op_mode		= OPMODE_I2C,
	.clock_speed	= 100000,
	.duty_cycle		= STD_DUTY_CYCLE
};

/// @brief Configuration for the I2C2 bus.
static const I2CConfig I2C2_CONFIG =
{
	.op_mode		= OPMODE_I2C,
	.clock_speed	= 100000,
	.duty_cycle		= STD_DUTY_CYCLE
};

/// @brief Configuration for the steering-angle-sensor's ADC driver.
static const as5600Config_t SAS_ADC_CONFIG =
{
	.addr		= 0x36,
	.i2c 		= &I2CD2,
	.sensor 	= (analogSensor_t*) &sas,
	.timeout 	= TIME_MS2I (100)
};

/// @brief Configuration for the ADC1 peripheral.
static const stmAdcConfig_t ADC_CONFIG =
{
	.driver = &ADCD1,
	.channels =
	{
		ADC_CHANNEL_IN10,	// APPS-1
		ADC_CHANNEL_IN11,	// APPS-2
		ADC_CHANNEL_IN12,	// BSE-F
		ADC_CHANNEL_IN13,	// BSE-R
		ADC_CHANNEL_IN0		// GLV Battery
	},
	.channelCount = 5,
	.sensors =
	{
		(analogSensor_t*) &pedals.apps1,
		(analogSensor_t*) &pedals.apps2,
		(analogSensor_t*) &pedals.bseF,
		(analogSensor_t*) &pedals.bseR,
		(analogSensor_t*) &glvBattery
	}
};

/// @brief Configuration for the on-board EEPROM.
static const mc24lc32Config_t PHYSICAL_EEPROM_CONFIG =
{
	.addr			= 0x50,
	.i2c			= &I2CD1,
	.timeout		= TIME_MS2I (500),
	.magicString	= EEPROM_MAP_STRING,
	.dirtyHook		= peripheralsReconfigure
};

/// @brief Configuration for the BMS's virtual EEPROM.
static const virtualEepromConfig_t VIRTUAL_EEPROM_CONFIG =
{
	.count		= 2,
	.entries	=
	{
		{
			.eeprom	= (eeprom_t*) &physicalEeprom,
			.addr	= 0x0000,
			.size	= 0x1000
		},
		{
			.eeprom	= &readonlyWriteonlyEeprom,
			.addr	= 0x1000,
			.size	= 0x1000
		}
	}
};

/// @brief Configuration for the GVL battery voltage measurment.
/// @note Minimum and maximum voltages are loaded from the EEPROM.
static linearSensorConfig_t glvBatteryConfig =
{
	.sampleMin	= 0,
	.sampleMax	= 4095,
	.valueMin	= 0,
	.valueMax	= 0
};

// Functions ------------------------------------------------------------------------------------------------------------------

bool peripheralsInit ()
{
	// I2C 1 driver initialization.
	if (i2cStart (&I2CD1, &I2C1_CONFIG) != MSG_OK)
		return false;

	// I2C 2 driver initalization.
	if (i2cStart (&I2CD2, &I2C2_CONFIG) != MSG_OK)
		return false;

	// ADC 1 driver initialization.
	if (!stmAdcInit (&adc, &ADC_CONFIG))
		return false;

	// Physical EEPROM initialization (only exit early if a failure occurred).
	if (!mc24lc32Init (&physicalEeprom, &PHYSICAL_EEPROM_CONFIG) && physicalEeprom.state == MC24LC32_STATE_FAILED)
		return false;

	// Read-only / Write-only EEPROM initialization.
	eepromInit (&readonlyWriteonlyEeprom, eepromWriteonlyWrite, eepromReadonlyRead);

	// Virtual memory initialization.
	virtualEepromInit (&virtualEeprom, &VIRTUAL_EEPROM_CONFIG);

	// Re-configurable peripherals are not considered fatal.
	peripheralsReconfigure (NULL);
	return true;
}

void peripheralsReconfigure (void* caller)
{
	(void) caller;

	// Pedals initialization
	pedalsInit (&pedals, &physicalEepromMap->pedalConfig);

	// SAS initialization
	sasInit (&sas, &physicalEepromMap->sasConfig);
	as5600Init (&sasAdc, &SAS_ADC_CONFIG);

	// Torque thread configuration
	torqueThreadSetDrivingTorqueLimit (physicalEepromMap->drivingTorqueLimit);
	torqueThreadSetRegenTorqueLimit (physicalEepromMap->regenTorqueLimit);
	torqueThreadSelectAlgorithm (physicalEepromMap->torqueAlgoritmIndex);
	torqueThreadSetPowerLimit (physicalEepromMap->powerLimit);
	torqueThreadSetPowerLimitPid
	(
		physicalEepromMap->powerLimitPidKp,
		physicalEepromMap->powerLimitPidKi,
		physicalEepromMap->powerLimitPidKd,
		physicalEepromMap->powerLimitPidA
	);

	// GLV battery initialization
	uint16_t glvSample11v5 = physicalEepromMap->glvBattery11v5;
	uint16_t glvSample14v4 = physicalEepromMap->glvBattery14v4;
	glvBatteryConfig.valueMin = lerp2d (0, glvSample11v5, 11.5f, glvSample14v4, 14.4f);
	glvBatteryConfig.valueMax = lerp2d (4095, glvSample11v5, 11.5f, glvSample14v4, 14.4f);
	linearSensorInit (&glvBattery, &glvBatteryConfig);
}

void peripheralsSample (systime_t timePrevious, systime_t timeCurrent)
{
	// Sample the pedal inputs and GLV battery.
	stmAdcSample (&adc);
	pedalsUpdate (&pedals, timePrevious, timeCurrent);

	if (physicalEepromMap->sasEnabled)
	{
		// If the SAS is enabled, sample the sensor.
		as5600Sample (&sasAdc);
	}
	else
	{
		// Otherwise, invalidate the sensor.
		sas.value = 0;
		sas.state = ANALOG_SENSOR_SAMPLE_INVALID;
	}

	bspdFault = palReadLine (LINE_BSPD_STATUS);
}