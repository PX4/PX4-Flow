#
# Board-specific definitions for the PX4FLOW
#

#
# Configure the toolchain
#

CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = PX4FLOW_V1

WUSEPACKED = -Wno-packed
include $(PX4_MK_DIR)baremetal/toolchain_gnu-arm-eabi.mk

