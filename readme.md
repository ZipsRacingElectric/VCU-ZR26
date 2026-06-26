# Vehicle Control Unit - Zips Racing ZR26

Embedded firmware for the vehicle control unit of ZR26. The VCU is the primary control unit of the vehicle, it is responsible for implementing the vehicle's state, torque distribution, and torque security.

## Usage

For help on how to setup this project, see the below file:

[Firmware Toolchain Setup Guide (common/doc/firmware_toolchain_guide.md)](https://github.com/ZipsRacingElectric/STM32F405-Common/blob/main/doc/firmware_toolchain_guide.md)

## Control Systems

### Torque Vectoring

One of the VCU's main responsibilities is implementing torque vectoring algorithms. For details about the VCU's bicycle model torque vectoring algorithm, see the below document:

[doc/bicycle_model.md](doc/bicycle_model.md)

For details about torque vectoring in general, or the implementation of such algorithms, see the below document:

[doc/torque_vectoring.md](doc/torque_vectoring.md)

### Power Limiting

Per the FSAE rules, the vehicle must obey an 80kW power limit. This VCU enforces this rule via a power limiter. For more details, see the below document:

[doc/power_limiting.md](doc/power_limiting.md)

## Calibration

In order for the VCU to work properly, it must be calibrated correctly. See the below document for details about calibration:

[doc/calibration.md](doc/calibration.md)

## Directory Structure

```
.
├── build                               - Directory for compilation output.
├── common                              - STM32 common library, see the readme in here for more details.
├── config                              - ChibiOS configuration files.
│   ├── board.chcfg                     - Defines the pin mapping and clock frequency of the board.
│   ├── chconf.h                        - ChibiOS RT configuration.
│   ├── halconf.h                       - ChibiOS HAL configuration.
│   ├── mcuconf.h                       - ChibiOS HAL driver configuration.
├── doc                                 - Documentation folder.
│   ├── chibios                         - ChibiOS documentation.
│   ├── datasheets                      - Datasheets of important components on this board.
│   ├── schematics                      - Schematics of this and related boards.
│   └── software                        - Software documentation.
├── makefile                            - Makefile for this application.
└── src                                 - C source / include files.
    ├── can                             - Code related to this device's CAN interface. This defines the messages this board
    │                                     transmits and receives.
    ├── controls                        - Code related to control systems. Torque vectoring implementations are defined here.
    └── peripherals                     - Code related to board hardware and peripherals.
```