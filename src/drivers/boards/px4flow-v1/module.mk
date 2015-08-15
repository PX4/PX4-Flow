#
# Board-specific startup code for the PX4FLOW
#

DEFAULT_VISIBILITY=default

ABS_BOOTLOADER_SRC := $(PX4_BOOTLOADER_BASE)src/

SRCS   = \
		   px4flow_init.c \

BOOTLOADER_SRC =  \
			$(ABS_BOOTLOADER_SRC)common/boot_app_shared.c \
 			$(ABS_BOOTLOADER_SRC)util/crc.c
		   
# Use the relitive path to keep the genrated files in the BUILD_DIR

SRCS += $(subst  $(PX4_MODULE_SRC),../../../,$(BOOTLOADER_SRC))
		   

MAXOPTIMIZATION	 = -Os
