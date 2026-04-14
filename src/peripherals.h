#ifndef PERIPHERALS_H
#define PERIPHERALS_H

// Peripherals ----------------------------------------------------------------------------------------------------------------
//
// Author: Cole Barach
// Date Created: 2024.09.29
//
// Description: Global objects representing the on-board hardware of the VCU.

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "peripherals/adc/analog_linear.h"
#include "peripherals/adc/stm_adc.h"

#include "peripherals/i2c/as5600.h"
#include "peripherals/i2c/mc24lc32.h"

#include "peripherals/eeprom_map.h"
#include "peripherals/pedals.h"

// Global Peripherals ---------------------------------------------------------------------------------------------------------

/// @brief ADC responsible for sampling all on-board analog inputs ( @c pedals & @c glvBattery ).
extern stmAdc_t adc;

/// @brief The VCU's physical (on-board) EEPROM. This is responsible for storing all non-volatile variables.
extern mc24lc32_t physicalEeprom;

/// @brief Structure mapping the EEPROM's contents to C datatypes.
static eepromMap_t* const physicalEepromMap = (eepromMap_t*) physicalEeprom.cache;

/// @brief The VCU's virtual memory map. This aggregates all externally accessible memory into a single map for CAN-bus access.
extern virtualEeprom_t virtualEeprom;

/// @brief Analog sensor measuring the voltage of the GLV battery.
extern linearSensor_t glvBattery;

/// @brief Analog sensors measuring the requests of the throttle and brake pedals.
extern pedals_t pedals;

/// @brief ADC measuring the steering-angle sensor.
extern as5600_t sasAdc;

/// @brief Sensor measuring the steering angle of the vehicle.
extern sas_t sas;

/// @brief Indicates whether the BSPD is fault or not.
extern bool bspdFault;

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the VCU's peripherals.
 * @return False if a fatal peripheral failed to initialize, true otherwise.
 */
bool peripheralsInit (void);

/**
 * @brief Re-initializes the VCU's peripherals after a change has been made to the on-board EEPROM.
 * @param caller Ignored. Used to make function signature compatible with EEPROM dirty hook.
 */
void peripheralsReconfigure (void* caller);

/**
 * @brief Samples all of the peripheral sensors. Must be done to update the values of the @c glvBattery , @c pedals , & @c sas
 * sensors.
 * @param timePrevious The time at which this function was last called (last value provided to @c timeCurrent ).
 * @param timeCurrent The current system time.
 */
void peripheralsSample (systime_t timePrevious, systime_t timeCurrent);

#endif // PERIPHERALS_H