#
# Makefile for the px4flow_default configuration
#

# Define the STMicroelectronics version to use

ST_LIB_VERSION=v1.0.2

INCLUDE_DIRS += $(PX4_BOOTLOADER_BASE)include \
			          $(ST_LIB_DIR)$(ST_LIB_VERSION) \
			          $(ST_LIB_DIR)$(ST_LIB_VERSION)/STM32F4xx_StdPeriph_Driver/inc \

#
# UAVCAN boot loadable Module ID
#

export UAVCANBLID_SW_VERSION_MAJOR=0
export UAVCANBLID_SW_VERSION_MINOR=1

#
# Bring in common uavcan hardware version definitions
#
include $(PX4_MK_DIR)baremetal/uavcan_board_px4flow-v1.mk

#
# Board support modules
#
MODULES		+= drivers/boards/px4flow-v1

#
# General system control
#
##TODO EXPECT I2C - MODULES		+= modules/uavcannode

# C support
MODULES		+= modules/libc

# Flow
MODULES		+= modules/flow



# Generate parameter XML file
#GEN_PARAM_XML = 1

