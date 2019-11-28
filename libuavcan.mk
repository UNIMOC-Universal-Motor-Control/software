# The original include.mk of libuavcan are broken for chibios makefile builds
# The origianl $(dir $(lastword $(MAKEFILE_LIST))) returns the path of the master makefile. 

LIBUAVCAN_DIR := ../../libuavcan/libuavcan

UAVCAN_DIR := $(LIBUAVCAN_DIR)/..

LIBUAVCAN_STM32_DIR := ../../libuavcan_stm32/driver

#
# Library sources
#
LIBUAVCAN_SRC := $(wildcard $(LIBUAVCAN_DIR)/src/*.cpp) $(wildcard $(LIBUAVCAN_STM32_DIR)/src/*.cpp)

LIBUAVCAN_INC := $(LIBUAVCAN_DIR)/include $(LIBUAVCAN_STM32_DIR)/include

#
# DSDL compiler executable
#
LIBUAVCAN_DSDLC := $(LIBUAVCAN_DIR)/dsdl_compiler/libuavcan_dsdlc

#
# Standard DSDL definitions
#
UAVCAN_DSDL_DIR := $(UAVCAN_DIR)/dsdl/uavcan
