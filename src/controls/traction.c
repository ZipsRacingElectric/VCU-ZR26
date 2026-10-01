// Header
#include "traction.h"

// Includes 
#include "peripherals.h"

#include "controls/vehicle_dynamics.h"

// Functions ---------------------------------------------------------------------------------------------------------------------------

void tractionStateUpdate(tractionState_t *state)
{
    for (uint8_t i = 0; i < AMK_COUNT; i++)
    {
        // Get motor speed of all 4 motors
        canNodeLock((canNode_t*) &amks[i]);
        float motorSpeed = amks[i].actualSpeed;
        canNodeUnlock((canNode_t*) &amks[i]);

        // Convert to ground speed, not accounting for tire slip
        state->wheelGroundSpeed[i] = motorSpeedToGroundSpeed(motorSpeed, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius);
    }
}