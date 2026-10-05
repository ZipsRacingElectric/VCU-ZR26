#ifndef TRACTION_H
#define TRACTION_H

// Vehicle Dynamics -----------------------------------------------------------------------------------------------------------
//
// Author: Jake Nowak
// Date Created: 2026.09.30
//
// Description: Data types and functions relating to tire traction to the ground.

// Includes -------------------------------------------------------------------------------------------------------------------

#include "can.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef enum
{
    VEHICLE_SPEED_IMU = 0,
    VEHICLE_SPEED_GPS = 1

} vehicleSpeedSource_t;

typedef struct
{   
    ///@brief Ground speed of all 4 wheels, not accounting for tire slip.
    float wheelGroundSpeed[AMK_COUNT];

    ///@brief Tire slip of all 4 wheels.
    float tireSlip[AMK_COUNT];

    ///@brief Vehicle speed estimated from integrating IMU accelertion.
    float integratedVehicleSpeed;

    /// @brief Vehicle speed estimated from GPS.
    float gpsVehicleSpeed;

    vehicleSpeedSource_t vehicleSpeedSource;

} tractionState_t;

// Function Prototypes --------------------------------------------------------------------------------------------------------

void tractionStateUpdate(tractionState_t* state, float deltaTime);

#endif // TRACTION_H