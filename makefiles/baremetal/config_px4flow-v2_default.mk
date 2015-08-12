#
# Makefile for the px4flow_default configuration
#

INCLUDE_DIRS += $(PX4_BOOTLOADER_BASE)include


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
MODULES		+= modules/uavcannode


# Generate parameter XML file
GEN_PARAM_XML = 1

#
# Make this UAVCAN boot loadable
#
MAKE_UAVCAN_BOOT_LOADABLE_ID=$(call MKUAVCANBLNAME,$(subst $\",,$(UAVCANBLID_NAME)))


