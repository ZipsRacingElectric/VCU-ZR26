// Header
#include "receive.h"

// Includes
#include "peripherals.h"
#include "can/can_thread.h"
#include "can/eeprom_can.h"

// Message IDs ----------------------------------------------------------------------------------------------------------------

#define EEPROM_COMMAND_MESSAGE_ID 0x010

// Receive Functions ----------------------------------------------------------------------------------------------------------

int8_t receiveMessage (void* configPtr, CANRxFrame* frame)
{
	canThreadConfig_t* config = configPtr;

	if (frame->SID == EEPROM_COMMAND_MESSAGE_ID)
	{
		eepromHandleCanCommand (frame, config->driver, (eeprom_t*) &virtualEeprom);
		return 0;
	}

	return -1;
}