// Header
#include "state_thread.h"

// Includes
#include "can.h"
#include "can/transmit.h"
#include "peripherals.h"

// ChibiOS
#include "hal.h"

// C Standard Library
#include "float.h"

// Constants ------------------------------------------------------------------------------------------------------------------

#define STATE_CONTROL_PERIOD	TIME_MS2I (10)
#define STATE_MESSAGE_PRESCALAR	10

#define BUZZER_TIME_PERIOD		TIME_MS2I (2500)
#define HV_INACTIVE_PERIOD		TIME_MS2I (100)

// Global Data ----------------------------------------------------------------------------------------------------------------

vehicleState_t vehicleState		= VEHICLE_STATE_LOW_VOLTAGE;
amkInverterState_t amksState	= AMK_STATE_ERROR;
bool torquePlausible			= true;
bool torqueDerating				= false;
bool vcuFault					= false;

float temperatureInverterMax;
float temperatureMotorMax;

// Thread Entrypoint ----------------------------------------------------------------------------------------------------------

static THD_WORKING_AREA (stateThreadWa, 512);
THD_FUNCTION (stateThread, arg)
{
	(void) arg;
	chRegSetThreadName ("state_control");

	systime_t timePrevious = chVTGetSystemTime ();
	systime_t timeoutBuzzer = timePrevious;
	systime_t timeoutHv = timePrevious;

	uint16_t messageCounter = 0;

	while (true)
	{
		systime_t timeCurrent = chVTGetSystemTime ();

		// If a failure occured previously, transition to LV and attempt to recover.
		if (vehicleState == VEHICLE_STATE_FAILED)
			vehicleState = VEHICLE_STATE_LOW_VOLTAGE;

		amksState = amksGetState (amks, AMK_COUNT);
		if (amksState == AMK_STATE_INVALID || physicalEeprom.state != MC24LC32_STATE_READY)
			vehicleState = VEHICLE_STATE_FAILED;

		canNodeLock ((canNode_t*) &bms);
		bool highVoltageEnabled = bms.state == CAN_NODE_VALID && bms.positiveIrClosed;
		canNodeUnlock ((canNode_t*) &bms);

		// Check vehicle state
		if (vehicleState == VEHICLE_STATE_LOW_VOLTAGE)
		{
			// Transition LV to HV
			if (highVoltageEnabled)
				vehicleState = VEHICLE_STATE_HIGH_VOLTAGE;
		}
		else if (vehicleState == VEHICLE_STATE_HIGH_VOLTAGE)
		{
			// Transition HV to RTD
			if (pedals.braking && !pedals.accelerating && !palReadLine (LINE_BUTTON_1_IN))
			{
				vehicleState = VEHICLE_STATE_READY_TO_DRIVE;
				palSetLine (LINE_BUZZER);
				timeoutBuzzer = chTimeAddX (timeCurrent, BUZZER_TIME_PERIOD);
			}

			// Check TS are still active
			if (highVoltageEnabled)
			{
				// Postpone the deadline if TS are active.
				timeoutHv = chTimeAddX (timeCurrent, HV_INACTIVE_PERIOD);
			}
			else if (chTimeIsInRangeX (timeCurrent, timePrevious, timeoutHv))
			{
				// Transition HV to LV if the deadline expires.
				vehicleState = VEHICLE_STATE_LOW_VOLTAGE;
			}
		}
		else if (vehicleState == VEHICLE_STATE_READY_TO_DRIVE)
		{
			// Check TS are still active
			if (highVoltageEnabled)
			{
				// Postpone the deadline if TS are active.
				timeoutHv = chTimeAddX (timeCurrent, HV_INACTIVE_PERIOD);
			}
			else if (!chTimeIsInRangeX (timeCurrent, timePrevious, timeoutHv))
			{
				// Transition RTD to LV if the deadline expires.
				vehicleState = VEHICLE_STATE_LOW_VOLTAGE;
			}
		}

		// Get cooling temps
		temperatureInverterMax	= -FLT_MAX;
		temperatureMotorMax		= -FLT_MAX;
		for (size_t index = 0; index < AMK_COUNT; ++index)
		{
			canNodeLock ((canNode_t*) &amks [index]);

			if (amks [index].temperatureInverter > temperatureInverterMax)
				temperatureInverterMax = amks [index].temperatureInverter;

			if (amks [index].temperatureMotor > temperatureMotorMax)
				temperatureMotorMax = amks [index].temperatureMotor;

			canNodeUnlock ((canNode_t*) &amks [index]);
		}

		// TODO(Barach): Enable if temp > EEPROM config value
		// Enable the cooling systems if high voltage is enabled.
		bool coolingEnabled = vehicleState == VEHICLE_STATE_HIGH_VOLTAGE || vehicleState == VEHICLE_STATE_READY_TO_DRIVE;
		palWriteLine (LINE_OUTPUT_1, coolingEnabled);
		palWriteLine (LINE_OUTPUT_2, coolingEnabled);

		// Stop the RTD buzzer if it is past the deadline.
		if (!chTimeIsInRangeX (timeCurrent, timePrevious, timeoutBuzzer))
			palClearLine (LINE_BUZZER);

		// Transmit the status message
		++messageCounter;
		if (messageCounter >= STATE_MESSAGE_PRESCALAR)
		{
			transmitStatusMessage (&CAND1, STATE_CONTROL_PERIOD);
			messageCounter = 0;
		}

		// VCU fault light
		vcuFault =
			!torquePlausible ||
			vehicleState == VEHICLE_STATE_FAILED;

		palWriteLine (LINE_LED_FAULT, vcuFault);

		// Brake light
		palWriteLine (LINE_BRAKE_LIGHT_CONTROL, pedals.braking);

		// Broadcast the sensor input message and debug message
		transmitSensorInputPercent (&CAND1, STATE_CONTROL_PERIOD);

		// Sleep until the next loop
		timePrevious = timeCurrent;
		systime_t timeNext = chTimeAddX (timeCurrent, STATE_CONTROL_PERIOD);
		chThdSleepUntilWindowed (timeCurrent, timeNext);
	}
}

// Functions ------------------------------------------------------------------------------------------------------------------

void stateThreadStart (tprio_t priority)
{
	// Start the torque control thread
	chThdCreateStatic (&stateThreadWa, sizeof (stateThreadWa), priority, stateThread, NULL);
}

void stateThreadSetTorquePlausibility (bool plausible, bool derating)
{
	// If implausible, set the fault indicator now.
	if (!plausible)
		palSetLine (LINE_LED_FAULT);

	torquePlausible = plausible;
	torqueDerating = derating;
}