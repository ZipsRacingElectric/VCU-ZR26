// Header
#include "can.h"

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "can/can_thread.h"
#include "can/receive.h"
#include "can/transmit.h"

// ChibiOS
#include "hal.h"

// C Standard Library
#include <string.h>

// Global Nodes ---------------------------------------------------------------------------------------------------------------

amkInverter_t	amks [AMK_COUNT];
bms_t			bms;
ecumasterGps_t	ecumaster;
sib_t			steeringInputBoard;

#define CAN1_NODE_COUNT (sizeof (can1Nodes) / sizeof (canNode_t*))
canNode_t* can1Nodes [] =
{
	(canNode_t*) &bms, (canNode_t*) &ecumaster,
	(canNode_t*) &amkRl, (canNode_t*) &amkRr, (canNode_t*) &amkFl, (canNode_t*) &amkFr
};

#define CAN2_NODE_COUNT (sizeof (can2Nodes) / sizeof (canNode_t*))
canNode_t* can2Nodes [] =
{
	(canNode_t*) &steeringInputBoard
};

// Configurations -------------------------------------------------------------------------------------------------------------

static const canThreadConfig_t CAN1_CONFIG =
{
	.name			= "can1_rx",
	.driver			= &CAND1,
	.period			= TIME_MS2I (10),
	.nodes			= can1Nodes,
	.nodeCount		= CAN1_NODE_COUNT,
	.rxHandler		= receiveMessage,
	.bridgeDriver	= NULL
};

static const canThreadConfig_t CAN2_CONFIG =
{
	.name			= "can2_rx",
	.driver			= &CAND2,
	.period			= TIME_MS2I (10),
	.nodes			= can2Nodes,
	.nodeCount		= CAN2_NODE_COUNT,
	.rxHandler		= NULL
};

#define CAN_TX_THREAD_PERIOD TIME_MS2I (250)

/**
 * @brief Configuration of the CAN 1 & CAN 2 peripherals.
 * @note See section 32.9 of the STM32F405 Reference Manual for more details.
 */
static const CANConfig CAN_DRIVER_CONFIG =
{
	.mcr = 	CAN_MCR_ABOM |		// Automatic bus-off management.
			CAN_MCR_AWUM |		// Automatic wakeup mode.
			CAN_MCR_TXFP,		// Chronologic FIFI priority.
	.btr =	CAN_BTR_SJW (0) |	// Max 1 TQ resynchronization jump.
			CAN_BTR_TS2 (1) |	// 2 TQ for time segment 2
			CAN_BTR_TS1 (10) |	// 11 TQ for time segment 1
			CAN_BTR_BRP (2)		// Baudrate divisor of 3 (1 Mbps)
};

static const amkInverterConfig_t AMK_CONFIGS [AMK_COUNT] =
{
	// RL
	{
		.mainDriver		= &CAND1,
		.bridgeDriver	= NULL,
		.baseId			= 0x200,
		.timeoutPeriod	= TIME_MS2I (100),
	},
	// RR
	{
		.mainDriver		= &CAND1,
		.bridgeDriver	= NULL,
		.baseId			= 0x201,
		.timeoutPeriod	= TIME_MS2I (100),
	},
	// FL
	{
		.mainDriver		= &CAND1,
		.bridgeDriver	= NULL,
		.baseId			= 0x202,
		.timeoutPeriod	= TIME_MS2I (100),
	},
	// FR
	{
		.mainDriver		= &CAND1,
		.bridgeDriver	= NULL,
		.baseId			= 0x203,
		.timeoutPeriod	= TIME_MS2I (100),
	}
};

static const bmsConfig_t BMS_CONFIG =
{
	.driver			= &CAND1,
	.timeoutPeriod	= TIME_MS2I (1000)
};

static const ecumasterGpsConfig_t ECUMASTER_CONFIG =
{
	.driver			= &CAND1,
	.timeoutPeriod	= TIME_MS2I (300),
};

static const sibConfig_t SIB_CONFIG =
{
	.driver			= &CAND2,
	.timeoutPeriod	= TIME_MS2I (500),
	.canId			= 0x405
};

// Threads --------------------------------------------------------------------------------------------------------------------

static CAN_THREAD_WORKING_AREA (can1RxThreadWa);

static CAN_THREAD_WORKING_AREA (can2RxThreadWa);

static THD_WORKING_AREA (can1TxThreadWa, 512);
THD_FUNCTION (can1TxThread, arg)
{
	(void) arg;
	chRegSetThreadName ("can1_tx");

	systime_t timeCurrent = chVTGetSystemTimeX ();
	while (true)
	{
		// Sleep until next loop.
		systime_t timeNext = chTimeAddX (timeCurrent, CAN_TX_THREAD_PERIOD);
		chThdSleepUntilWindowed (timeCurrent, timeNext);
		timeCurrent = chVTGetSystemTimeX ();

		transmitTemperaturesMessage (&CAND1, CAN_TX_THREAD_PERIOD);
		transmitConfigMessage (&CAND1, CAN_TX_THREAD_PERIOD);
	}
}

// Functions ------------------------------------------------------------------------------------------------------------------

bool canInterfaceInit (tprio_t priority)
{
	// CAN 1 driver initialization
	if (canStart (&CAND1, &CAN_DRIVER_CONFIG) != MSG_OK)
		return false;
	palClearLine (LINE_CAN1_STBY);

	// CAN 2 driver initialization
	if (canStart (&CAND2, &CAN_DRIVER_CONFIG) != MSG_OK)
		return false;
	palClearLine (LINE_CAN2_STBY);

	// Initialize the CAN nodes
	for (uint8_t index = 0; index < AMK_COUNT; ++index)
		amkInit (amks + index, AMK_CONFIGS + index);
	bmsInit (&bms, &BMS_CONFIG);
	ecumasterInit (&ecumaster, &ECUMASTER_CONFIG);
	sibInit (&steeringInputBoard, &SIB_CONFIG);

	// Create the CAN 1 RX thread
	canThreadStart (can1RxThreadWa, sizeof (can1RxThreadWa), priority, &CAN1_CONFIG);

	// Create the CAN 2 RX thread
	canThreadStart (can2RxThreadWa, sizeof (can2RxThreadWa), priority, &CAN2_CONFIG);

	// Create the CAN 1 TX thread
	chThdCreateStatic (&can1TxThreadWa, sizeof (can1TxThreadWa), LOWPRIO, can1TxThread, NULL);

	return true;
}