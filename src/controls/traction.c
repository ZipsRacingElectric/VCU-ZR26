// Header
#include "traction.h"

// Includes 
#include "peripherals.h"

#include "controls/vehicle_dynamics.h"

// Functions ---------------------------------------------------------------------------------------------------------------------------

static void tractionIntegrationSpeed(tractionState_t* state, float longitudinalAcceleration, float deltaTime)
{
    state->integratedVehicleSpeed += longitudinalAcceleration * deltaTime;
}  

static void calculateTireSlip(tractionState_t* state)
{
    float IMU = state->integratedVehicleSpeed;
    float GPS = state->gpsVehicleSpeed;

    // If IMU speed is faster than GPS speed set vehicle speed, else vehicle speed equals GPS speed
    //float vehicleSpeed = (IMU > GPS) ? IMU : GPS;

    bool useImu = IMU > GPS;

    float vehicleSpeed = useImu ? IMU : GPS;

    state->vehicleSpeedSource = useImu ? VEHICLE_SPEED_IMU : VEHICLE_SPEED_GPS;

    for (uint8_t wheel = 0; wheel < AMK_COUNT; wheel++) 
    {
        float groundSpeed = state->wheelGroundSpeed[wheel];
        // Wheels slipping, but car has zero velocity
        if (vehicleSpeed == 0)
        {
            state->tireSlip[wheel] = 0.0f;
        }
        else 
        {
            // Caclculate tire slip
            state->tireSlip[wheel] = (groundSpeed - vehicleSpeed) / (vehicleSpeed);
        }
    }
}

void tractionStateUpdate(tractionState_t* state, float deltaTime)
{
    // Get longitudinal accleration from IMU 
    canNodeLock((canNode_t*) &boschImu);
        float xAcceleration = boschImu.xAcceleration;
    canNodeUnlock((canNode_t*) &boschImu);

    // Convert acceleration from g to m/s^2
    float longitudinalAcceleration = G_TO_MPS2(xAcceleration);

    // Integrate acceleration to estimate vehicle speed in m/s
    tractionIntegrationSpeed(state, longitudinalAcceleration, deltaTime);

     // Get vehicle speed from GPS
    canNodeLock((canNode_t*) &ecumaster);
    float gpsSpeed = ecumaster.speed;
    canNodeUnlock((canNode_t*) &ecumaster);

    // Convert GPS speed from km/h to m/s
    state->gpsVehicleSpeed = KPH_TO_MPS(gpsSpeed);

    for (uint8_t i = 0; i < AMK_COUNT; i++)
    {
        // Get motor speed of all 4 motors
        canNodeLock((canNode_t*) &amks[i]);
        float motorSpeed = amks[i].actualSpeed;
        canNodeUnlock((canNode_t*) &amks[i]);

        // Convert to ground speed, not accounting for tire slip
        float tempSpeed = motorSpeedToGroundSpeed (motorSpeed, physicalEepromMap->gearRatio, physicalEepromMap->wheelRadius);
        state->wheelGroundSpeed[i] = KPH_TO_MPS(tempSpeed);
    }

    calculateTireSlip(state);
}