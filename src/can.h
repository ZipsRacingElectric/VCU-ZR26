#ifndef CAN_H
#define CAN_H

// CAN Thread -----------------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.09.29
//
// Description: Objects related the the VCU's CAN interface. The VCU uses 2 CAN busses.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "can/amk_inverter.h"
#include "can/bms.h"
#include "can/ecumaster_gps_v2.h"
#include "can/steering_input_board.h"

// ChibiOS
#include "ch.h"

// Constants ------------------------------------------------------------------------------------------------------------------

/// @brief The number of AMK inverters on the CAN bus.
#define AMK_COUNT 4

// CAN 1 Global Nodes ---------------------------------------------------------------------------------------------------------

/// @brief The rear-left AMK inverter.
#define amkRl (amks [0])

/// @brief The rear-right AMK inverter.
#define amkRr (amks [1])

/// @brief The front-left AMK inverter.
#define amkFl (amks [2])

/// @brief The front-right AMK inverter.
#define amkFr (amks [3])

/// @brief Array of all the AMK inverters.
extern amkInverter_t amks [AMK_COUNT];

/// @brief The accumulator's battery management system.
extern bms_t bms;

/// @brief The ECUMaster GPS/IMU.
extern ecumasterGps_t ecumaster;

// CAN 2 Global Nodes ---------------------------------------------------------------------------------------------------------

/// @brief The steering input board.
extern sib_t steeringInputBoard;

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes both of the VCU's CAN interfaces.
 * @param priority The priority to start the CAN threads at.
 * @return False if a fatal error occurred, true otherwise.
 */
bool canInterfaceInit (tprio_t priority);

#endif // CAN_H