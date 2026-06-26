# VCU Sensor Calibration

The VCU has several sensors that need to be calibrated after it is installed in a vehicle. This guide details how to perform said calibration.

## APPS (Accelerator Pedal Position Sensors)

The APPS have two methods of calibration. The first method is calibrating the analog output of the sensor via 3rd party software. The second method of calibration is mapping the analog output of said sensor to throttle positions.

The first calibration method (Reventec-Side) is required when either:
- A new reventec sensor is installed on a pedal.
- A new pedal is fitted with a sensor.

The second calibration method (VCU-Side) is required periodically to compensate for sensor drift (or anytime the Reventec-Side calibration is performed).

### Reventec-Side Calibration

The Reventec sensor calibration is performed via Reventec's 3rd party software. The guide on how to perform this calibration and the software itself can be found in the team's OneDrive:

`ZR26/Energetics/Software/Software Tools/Reventec`

The Channel 1 sensor should be calibrated to output 0.5V to 4.5V, where 0.5V is the released position and 4.5V is the fully depressed position.

The Channel 2 sensor should be calibrated to output 0.25V to 2.25V, where 0.25V is the released position and 2.25V is the fully depressed position.

### VCU-Side Calibration

After the Reventec Calibration has been performed, or anytime the sensor's output drifts, the VCU APPS calibration must be performed. This is done via ZRE-CAN-Tools, specifically, the `eeprom-cli`.

Via the `eeprom-cli`:
- Bring the throttle to the fully released position
	- Use the `m` option to read the `APPS_1_SAMPLE` and `APPS_2_SAMPLE` values.
		- They should read somewhere around 250 and 125, respectively.
	- Program the `APPS_1_ABS_MIN` and `APPS_2_ABS_MIN` values to be ~50% of the measured values.

- Bring the throttle to the fully pressed position
	- Use the `m` option to read the `APPS_1_SAMPLE` and `APPS_2_SAMPLE` values.
		- They should read somewhere around 3300 and 1650, respectively.
	- Program the `APPS_1_ABS_MAX` and `APPS_2_ABS_MAX` values to be ~110% of the measured values.

- Bring the throttle to the fully released position, then press it to ~10% of the travel
	- While holding the throttle in this position, use the `m` option to read the `APPS_1_SAMPLE` and `APPS_2_SAMPLE` values.
	- Program the `APPS_1_REQ_MIN` and `APPS_2_REQ_MIN` values to be exactly the measured values.

- Bring the throttle to the fully pressed position, then release it to ~90% of the travel
	- While holding the throttle in this position, use the `m` option to read the `APPS_1_SAMPLE` and `APPS_2_SAMPLE` values.
	- Program the `APPS_1_REQ_MAX` and `APPS_2_REQ_MAX` values to be exactly the measured values.

After this, the calibration should be complete. Use the `dashboard-gui` to validate the `APPS_1_PERCENT` and `APPS_2_PERCENT` signals move correctly from 0% to 100% over the pedal travel.
- There should be a slight deadzone at the beginning and end of the pedal travel
- Both signals must be 0% when the pedal is not being pressed (it cannot flicker at all)
- Over the entire range of travel, the `APPS_1_PERCENT` and `APPS_2_PERCENT` signals cannot differ by more than 10%.

## BSE (Brake System Encoder)

The BSE only have a single method of calibration (the VCU-side calibration of the APPS). This is done via ZRE-CAN-Tools, specifically, the `eeprom-cli`.

Via the `eeprom-cli`:
- Fully release the brake pedal
	- Use the `m` option to read the `BSE_F_SAMPLE` and `BSE_R_SAMPLE` values.
	- Program the `BSE_F_ABS_MIN` and `BSE_R_ABS_MIN` values to be ~50% of the measured values.

- Slightly press the brake pedal (harder than just resting your foor on it)
	- Use the `m` option to read the `BSE_F_SAMPLE` and `BSE_R_SAMPLE` values.
	- Program the `BSE_F_REQ_MIN` and `BSE_R_REQ_MIN` values to be exactly the measured values.

- The highest brake pressure is impossible to actually reach by foot, here we just use an estimate.
	- Program the `BSE_F_REQ_MAX` and `BSE_R_REQ_MAX` values to 3900.
	- Program the `BSE_F_ABS_MAX` and `BSE_R_ABS_MAX` values to 4000.

## SAS (Steering Angle Sensor)

The SAS needs recalibrated any time the steering rack is disassembled or vehicle toe is adjusted. This is to keep to 0 degree value inline with straight steering. This is done via ZRE-CAN-Tools, specifically, the `eeprom-cli`.

**Note: The angle of the wheel being straight and the vehicle's tires being straight are not always the same. The steering angle sensor's output is defined to be the angle of the steering wheel offset by the difference in these two positions. The steering angle sensor should always read 0 degrees when the tires are pointed straight.**

Via the `eeprom-cli`:
- Point the vehicle's tires straight.
	- Use the `m` option to read the `SAS_SAMPLE` value.
	- Program the `SAS_SAMPLE_ZERO` value to be exactly the measured value.
	- Measure the angle of the steering wheel and note this as the zero'ed angle.

- Steer the wheel completely to the left (counter-clockwise).
	- Use the `m` option to read the `SAS_SAMPLE` value.
	- Program the `SAS_SAMPLE_NEGATIVE` value to be exactly the measured value.
	- Program the `SAS_ANGLE_NEGATIVE` value to be the difference between the zero'ed angle of the wheel and the current angle of the wheel. This should be a negative value.

- Steer the wheel completely to the right (clockwise).
	- Use the `m` option to read the `SAS_SAMPLE` value.
	- Program the `SAS_SAMPLE_POSITIVE` value to be exactly the measured value.
	- Program the `SAS_ANGLE_POSITIVE` value to be the difference between the current angle of the wheel and the zero'ed angle of the wheel. This should be a positive value.

The `SAS_ANGLE_NEGATIVE` and `SAS_ANGLE_POSITIVE` values should be very close in magnitude (assuming a symmetrical steering rack). If they differ by a significant amount, recheck your measurments or consult someone from VD.

After this, sweep the steering wheel from left-to-right and use the `dashboard-gui` to validate the `STEERING_ANGLE` value moves as expected.

## GLV Battery Voltage

The GLV battery voltage measurement needs calibrated once per-PCB. This is best done on the test bench due to it having a variable power supply.

Via the `eeprom-cli`:
- Set the power supply to 20V
	- Use the `m` option to read the `GLV_BATTERY_SAMPLE` value.
	- Program the `GLV_BATTERY_SAMPLE_20V` value to this value.

- Set the power supply to 29V
	- Use the `m` option to read the `GLV_BATTERY_SAMPLE` value.
	- Program the `GLV_BATTERY_SAMPLE_29V` value to this value.

After this, use the `dashboard-gui` to validate the `GLV_BATTERY_VOLTAGE` value matches the power supply output between the voltages of 20V and 29V.