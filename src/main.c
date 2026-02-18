// Vehicle Control Unit -------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.09.21
//
// Description: Entrypoint and interrupt handling for the vehicle control unit. See the respective threads for specific
//   responsibilities of the the VCU.

// TODO(Barach): Hybrid watchdog

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "debug.h"
#include "can.h"
#include "peripherals.h"
#include "state_thread.h"
#include "torque_thread.h"

// ChibiOS
#include "hal.h"

// Interrupts -----------------------------------------------------------------------------------------------------------------

void hardFaultCallback (void)
{
	// Blink the fault LED
	while (true)
	{
		palToggleLine (LINE_LED_FAULT);
		while (DWT->CYCCNT % 16800000 != 0);
	}
}

// Entrypoint -----------------------------------------------------------------------------------------------------------------

int main (void)
{
	// ChibiOS Initialization
	halInit ();
	chSysInit ();

	// Debug initialization
	ioline_t ledLine = LINE_LED_HEARTBEAT;
	debugHeartbeatStart (&ledLine, LOWPRIO);

	// Peripheral initialization
	if (!peripheralsInit ())
	{
		hardFaultCallback ();
		while (true);
	}

	// CAN initialization. Start this first as to invalidate all can nodes before any other threads attempt reading
	// any data.
	if (!canInterfaceInit (NORMALPRIO))
	{
		hardFaultCallback ();
		while (true);
	}

	// Torque thread initialization. Start this at the highest priority as it has the strictest timing.
	torqueThreadStart (NORMALPRIO + 1);

	// State thread initialization. Start this at a lower priority as it has the least strict timing.
	stateThreadStart (NORMALPRIO - 1);

	// Allow the shutdown loop to close.
	palSetLine (LINE_SHUTDOWN_CONTROL);

	// Do nothing.
	while (true)
		chThdSleepMilliseconds (500);
}