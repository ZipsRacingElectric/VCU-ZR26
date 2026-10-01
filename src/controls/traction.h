#ifndef TRACTION_H
#define TRACTION_H

// Vehicle Dynamics -----------------------------------------------------------------------------------------------------------
//
// Author: Jake Nowak
// Date Created: 2026.09.30
//
// Description: Data types relating to tire traction to the ground.

// Includes -------------------------------------------------------------------------------------------------------------------

#include "can/amk_inverter.h"
#include "can.h"

// Datatypes ------------------------------------------------------------------------------------------------------------------

typedef struct
{   
    ///@brief Ground speed of all 4 wheels, not accounting for tire slip
    float wheelGroundSpeed[AMK_COUNT];
    
//    ///@brief Tire slip of all 4 wheels
//    float slip[AMK_COUNT];

} tractionState_t;

// Function Prototypes --------------------------------------------------------------------------------------------------------

void tractionStateUpdate(tractionState_t* state);

#endif // TRACTION_H