# Project name
PROJECT = vcu_zr26

# Imported files
CHIBIOS  := $(CHIBIOS_SOURCE_PATH)

# Directories
CONFDIR  	:= ./config
BUILDDIR 	:= ./build
DEPDIR   	:= ./build/dep
BOARDDIR	:= ./build/board
COMMONDIR	:= ./common

# Includes
ALLINC += src

# Source files
CSRC =	$(ALLCSRC)							\
		src/main.c							\
											\
		src/peripherals.c					\
		src/peripherals/eeprom_map.c		\
		src/peripherals/pedals.c			\
		src/peripherals/steering_angle.c	\
											\
		src/can.c							\
		src/can/receive.c					\
		src/can/transmit.c					\
											\
		src/torque_thread.c					\
		src/controls/tv_const_bias.c		\
		src/controls/tv_linear_bias.c		\
		src/controls/tv_bicycle_model.c		\
											\
		src/state_thread.c

# Common library includes
include common/src/debug.mk
include common/src/fault_handler.mk

include common/src/peripherals/adc/analog_linear.mk
include common/src/peripherals/adc/stm_adc.mk
include common/src/peripherals/i2c/as5600.mk
include common/src/peripherals/i2c/mc24lc32.mk

include common/src/can/amk_inverter.mk
include common/src/can/bms.mk
include common/src/can/can_node.mk
include common/src/can/can_thread.mk
include common/src/can/ecumaster_gps_v2.mk
include common/src/can/eeprom_can.mk

include common/src/controls/pid_controller.mk

# Compiler flags
USE_OPT = -Og -Wall -Wextra -lm

# C macro definitions
UDEFS =

# ASM definitions
UADEFS =

# Include directories
UINCDIR =

# Library directories
ULIBDIR =

# Libraries
ULIBS =

# Common toolchain includes
include common/common.mk
include common/make/openocd.mk

# ChibiOS compilation hooks
PRE_MAKE_ALL_RULE_HOOK: $(BOARD_FILES) $(CLANGD_FILE)