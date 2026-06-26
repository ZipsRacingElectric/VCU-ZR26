# Torque Vectoring

The VCU implements "hot-swappable" torque vectoring algorithms. The `TORQUE_ALGORITHM_INDEX` EEPROM variable determines what algorithm is selected.

## Torque Vectoring Algorithm Implementations

The [../src/controls/torque_vectoring.h](../src/controls/torque_vectoring.h) file defines the interface torque vectoring algorithms must implement.

Summarized, a torque vectoring algorithm can be described as the combination of a function, a configuration, and a state.

The configuration defines the list of "tunable" parameters. These may be the coefficients of a PID or a constant biasing on how to distribute torque. Every torque vectoring algorithm defines a custom configuration based on its needs. Often, the configuration is stored in the VCU's EEPROM, meaning it can be changed via the `eeprom-cli`.

The state defines the list of information that is "stored" between different iterations of the algorithm. A PID controller for instance has a state that must be preserved, so any algorithm using a PID controller must include the PID in its state. Derivatives, integrals, and filters are all examples of components that have states associated with them.

The function is the main part of an algorithm. It is simply the code that is executed to "compute" the output of the algorithm. Every torque vectoring algorithm function "looks" the same from the outside, meaning it has the same set of inputs and outputs. The inputs and outputs are summarized later.

### Control System Frequency

The torque control thread runs at a target frequency of 100 Hz. This is not guaranteed to be achieved, but the actual rate should be quite close to it. The `deltaTime` input to the algorithm will specify the actual amount of time that has ellapsed since the last call to the algorithm's function.

### Inputs

A general set of inputs is provided for all torque vectoring algorithms. Note that inputs are only the information that is specialized to torque vectoring, sensor information is not an input, but is still available to the function. See the *List of Sensors* section for a list of usable non-inputs.

- `deltaTime` - The amount of time elapsed between now and the previous call to the algorithm's update function, in seconds.
- `torqueRequest` - The total amount of torque the algorithm is to provide. Positive for driving torque, negative for regen torque. While it should be respected if possible, the actual torques request may sum to a magnitude less than this due to saturation.
- `drivingTorqueLimitRl` - The maximum requestable amount of driving torque to the RL motor. Any more will be saturated.
- `drivingTorqueLimitRr` - The maximum requestable amount of driving torque to the RR motor. Any more will be saturated.
- `drivingTorqueLimitFl` - The maximum requestable amount of driving torque to the FL motor. Any more will be saturated.
- `drivingTorqueLimitFr` - The maximum requestable amount of driving torque to the FR motor. Any more will be saturated.
- `regenTorqueLimitRl` - The maximum requestable amount of regen torque to the RL motor. Any more will be saturated.
- `regenTorqueLimitRr` - The maximum requestable amount of regen torque to the RR motor. Any more will be saturated.
- `regenTorqueLimitFl` - The maximum requestable amount of regen torque to the FL motor. Any more will be saturated.
- `regenTorqueLimitFr` - The maximum requestable amount of regen torque to the FR motor. Any more will be saturated.
- `drivingFrBias` - The front-to-rear bias for distributing driving torque. Not required for all torque vectoring algorithms. 1 => 100% rearwards, 0 => 100% frontwards, 0.5 => 50:50 split.

- `regenFrBias` - The front-to-rear bias for distributing regen torque. Not required for all torque vectoring algorithms. 1 => 100% rearwards, 0 => 100% frontwards, 0.5 => 50:50 split.

### Outputs

- `valid` - Output request validity. Indicates whether the the output of this algoritm is usable.
- `torqueRl` - The torque to request to the rear-left motor. Positive for driving torque, negative for regen torque.
- `torqueRr` - The torque to request to the rear-right motor. Positive for driving torque, negative for regen torque.
- `torqueFl` - The torque to request to the front-left motor. Positive for driving torque, negative for regen torque.
- `torqueFr` - The torque to request to the front-right motor. Positive for driving torque, negative for regen torque.

## Sensors

The table below provides a list of all notable sensor data available to the VCU. This does not include all potential sources of information, just what is currently available.

| Sensor        | Variable Name              | Description                                  | Unit               |
|---------------|----------------------------|----------------------------------------------|--------------------|
| APPS          | `pedals.appsRequest`       | Throttle Position                            | Ratio [0, 1]       |
| BSE           | `pedals.bseRequest`        | Brake position                               | Ratio [0, 1]       |
| SAS           | `sas.value`                | Steering wheel angle                         | Degrees            |
| RL AMK        | `amkRl.actualTorque`       | Actual delivered torque by RL motor          | Nm                 |
|               | `amkRl.actualSpeed`        | Actual speed of the RL motor                 | RPM                |
| RR AMK        | `amkRr.actualTorque`       | Actual delivered torque by RR motor          | Nm                 |
|               | `amkRr.actualSpeed`        | Actual speed of the RR motor                 | RPM                |
| FL AMK        | `amkFl.actualTorque`       | Actual delivered torque by FL motor          | Nm                 |
|               | `amkFl.actualSpeed`        | Actual speed of the FL motor                 | RPM                |
| FR AMK        | `amkFr.actualTorque`       | Actual delivered torque by FR motor          | Nm                 |
|               | `amkFr.actualSpeed`        | Actual speed of the FR motor                 | RPM                |
| ECUMaster GPS | `ecumaster.speed`          | Vehicle reference speed                      | km/h               |
|               | `ecumaster.xAcceleration`  | x-axis acceleration                          | g's                |
|               | `ecumaster.yAcceleration`  | y-axis acceleration                          | g's                |
|               | `ecumaster.zAcceleration`  | z-axis acceleration                          | g's                |
| Bosch IMU     | `boschImu.xAngleRate`      | x-axis angle rate                            | deg/s              |
|               | `boschImu.yAngleRate`      | y-axis angle rate                            | deg/s              |
|               | `boschImu.zAngleRate`      | z-axis angle rate                            | deg/s              |
|               | `boschImu.xAcceleration`   | x-axis acceleration                          | g's                |
|               | `boschImu.yAcceleration`   | y-axis acceleration                          | g's                |
|               | `boschImu.zAcceleration`   | z-axis acceleration                          | g's                |
| SIB           | `sib.buttonsHeld []`       | Steering wheel buttons                       | True/false         |
|               | `sib.analogValues []`      | Steering wheel paddles                       | Ratio [0, 1]       |

Note that just because there is information available for a sensor, it does not mean said data is valid. This is especially true for CAN bus sensors, as there is no guarantee said device is activly reporting CAN data. For any source of information, a check should be performed to ensure said data is valid. The table below provides the list of checks to perform for each sensor.

| Sensor        | Validity Check                       |
|---------------|--------------------------------------|
| APPS          | `pedals.plausible`                   |
| BSE           | `pedals.plausible`                   |
| SAS           | `sas.state == ANALOG_SENSOR_VALID`   |
| RL AMK        | `amkRl.state == CAN_NODE_VALID`      |
| RR AMK        | `amkRr.state == CAN_NODE_VALID`      |
| FL AMK        | `amkFl.state == CAN_NODE_VALID`      |
| FR AMK        | `amkFr.state == CAN_NODE_VALID`      |
| ECUMaster GPS | `ecumaster.state == CAN_NODE_VALID`  |
| Bosch IMU     | `boschImu.state == CAN_NODE_VALID`   |
| SIB           | `sib.state == CAN_NODE_VALID`        |

Also note that, when using CAN bus sensors (any sensor that implements the `canNode_t` interface), the node's mutex must be locked in order to get both the validity and the sensor value.