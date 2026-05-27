// Header
#include "eeprom_map.h"

// Includes
#include "can.h"
#include "peripherals.h"
#include "torque_thread.h"

// C Standard Library
#include <string.h>

// Global Data ----------------------------------------------------------------------------------------------------------------

#define READONLY_COUNT (sizeof (READONLY_ADDRS) / sizeof (READONLY_ADDRS [0]))
static const uint16_t READONLY_ADDRS [] =
{
	0x0000,
	0x0002,
	0x0004,
	0x0006,
	0x0008,
	0x000A,
	0x000C,
	0x0010,
	0x0014,
	0x0018
};

static const void* READONLY_DATA [READONLY_COUNT] =
{
	&pedals.apps1.sample,
	&pedals.apps2.sample,
	&pedals.bseF.sample,
	&pedals.bseR.sample,
	&sas.sample,
	&glvBattery.sample,
	&torqueRequestNonDerated.torqueRl,
	&torqueRequestNonDerated.torqueRr,
	&torqueRequestNonDerated.torqueFl,
	&torqueRequestNonDerated.torqueFr
};

static const uint16_t READONLY_SIZES [READONLY_COUNT] =
{
	sizeof (pedals.apps1.sample),
	sizeof (pedals.apps2.sample),
	sizeof (pedals.bseF.sample),
	sizeof (pedals.bseR.sample),
	sizeof (sas.sample),
	sizeof (glvBattery.sample),
	sizeof (torqueRequestNonDerated.torqueRl),
	sizeof (torqueRequestNonDerated.torqueRr),
	sizeof (torqueRequestNonDerated.torqueFl),
	sizeof (torqueRequestNonDerated.torqueFr)
};

// Functions ------------------------------------------------------------------------------------------------------------------

bool eepromReadonlyRead (void* object, uint16_t addr, void* data, uint16_t dataCount)
{
	(void) object;

	for (uint16_t index = 0; index < READONLY_COUNT; ++index)
	{
		if (addr != READONLY_ADDRS [index])
			continue;

		if (dataCount != READONLY_SIZES [index])
			return false;

		memcpy (data, READONLY_DATA [index], dataCount);
		return true;
	}

	return false;
}

bool eepromWriteonlyWrite (void* object, uint16_t addr, const void* data, uint16_t dataCount)
{
	(void) object;
	(void) data;
	(void) dataCount;

	switch (addr)
	{
	case 0x0000: // AMK_RESET_REQUEST
		for (uint8_t index = 0; index < AMK_COUNT; ++index)
			for (uint8_t i = 0; i < 10; ++i)
				amkSendErrorResetRequest (amks + index, TIME_MS2I (100));
		return true;
	}

	return false;
}