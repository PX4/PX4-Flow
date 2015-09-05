#
#   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

#
# Top-level Makefile for building PX4 firmware images.
#
LEGACY_ELF = $(BUILD_DIR)px4flow-v1_default.build/firmware.elf
LEGACY_PX4 = $(IMAGE_DIR)px4flow-v1_default.px4

export EXTRADEFINES += -DNO_PRINT_TODOS

TARGETS	:= baremetal
EXPLICIT_TARGET	:= $(filter $(TARGETS),$(MAKECMDGOALS))
ifneq ($(EXPLICIT_TARGET),)
    export PX4_TARGET_OS=$(EXPLICIT_TARGET)
    export GOALS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
endif

#
# Get path and tool configuration
#
export PX4_BASE		 := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))/
include $(PX4_BASE)makefiles/setup.mk

#
# Get a version string provided by git
# This assumes that git command is available and that
# the directory holding this file also contains .git directory
#
GIT_DESC := $(shell git log -1 --pretty=format:%H)
ifneq ($(words $(GIT_DESC)),1)
    GIT_DESC := "unknown_git_version"
endif

export GIT_DESC

GIT_DESC_SHORT := $(shell echo $(GIT_DESC) | cut -c1-16)

#
# Canned firmware configurations that we (know how to) build.
#
KNOWN_CONFIGS		:= $(subst config_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)/$(PX4_TARGET_OS)/config_*.mk))))
CONFIGS			?= $(KNOWN_CONFIGS)

#
# Boards that we (know how to) build NuttX export kits for.
#
KNOWN_BOARDS		:= $(subst board_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)/$(PX4_TARGET_OS)/board_*.mk))))
BOARDS			?= $(KNOWN_BOARDS)

#
# Canned firmware configurations for bootloader that we (know how to) build.
#
KNOWN_BOARDS_WITH_BOOTLOADERS		:=  $(subst _bootloader,, $(filter %_bootloader, $(CONFIGS)))
BOARDS_WITH_BOOTLOADERS	:= $(filter $(BOARDS), $(KNOWN_BOARDS_WITH_BOOTLOADERS))

#
# Debugging
#
MQUIET			 = --no-print-directory
#MQUIET			 = --print-directory

################################################################################
# No user-serviceable parts below
################################################################################

#
# If the user has listed a config as a target, strip it out and override CONFIGS.
#
FIRMWARE_GOAL		 = firmware
EXPLICIT_CONFIGS	:= $(filter $(CONFIGS),$(MAKECMDGOALS))
ifneq ($(EXPLICIT_CONFIGS),)
CONFIGS			:= $(EXPLICIT_CONFIGS)
.PHONY:			$(EXPLICIT_CONFIGS)
$(EXPLICIT_CONFIGS):	all

#
# If the user has asked to upload, they must have also specified exactly one
# config.
#
ifneq ($(filter upload,$(MAKECMDGOALS)),)
ifneq ($(words $(EXPLICIT_CONFIGS)),1)
$(error In order to upload, exactly one board config must be specified)
endif
FIRMWARE_GOAL		 = upload
.PHONY: upload
upload:
	@:
endif
endif

ifeq ($(PX4_TARGET_OS),baremetal)
include $(PX4_BASE)makefiles/baremetal/firmware_baremetal.mk
endif

#
# Versioning
#

GIT_VER_FILE = $(PX4_VERSIONING_DIR).build_git_ver
GIT_HEADER_FILE = $(PX4_VERSIONING_DIR)build_git_version.h

$(GIT_VER_FILE) :
	$(Q) if [ ! -f $(GIT_VER_FILE) ]; then \
		$(MKDIR) -p $(PX4_VERSIONING_DIR); \
		$(ECHO) "" > $(GIT_VER_FILE); \
	fi

.PHONY: checkgitversion
checkgitversion: $(GIT_VER_FILE)
	$(Q) if [ "$(GIT_DESC)" != "$(shell cat $(GIT_VER_FILE))" ]; then \
		$(ECHO) "/* Auto Magically Generated file */" > $(GIT_HEADER_FILE); \
		$(ECHO) "/* Do not edit! */" >> $(GIT_HEADER_FILE); \
		$(ECHO) "#define PX4_GIT_VERSION_STR  \"$(GIT_DESC)\"" >> $(GIT_HEADER_FILE); \
		$(ECHO) "#define PX4_GIT_VERSION_BINARY 0x$(GIT_DESC_SHORT)" >> $(GIT_HEADER_FILE); \
		$(ECHO) $(GIT_DESC) > $(GIT_VER_FILE); \
	fi
#
# Sizes
#

.PHONY: size
size:
	$(Q) for elfs in Build/*; do if [ -f  $$elfs/firmware.elf ]; then  $(SIZE) $$elfs/firmware.elf; fi done

#
# Submodule Checks
#

.PHONY: checksubmodules
checksubmodules:
	$(Q) ($(PX4_BASE)/Tools/check_submodules.sh)

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

#
# Testing targets
#
testbuild:
	$(Q) (cd $(PX4_BASE) && $(MAKE) distclean && $(MAKE) archives && $(MAKE) -j8)
	$(Q) (zip -r Firmware.zip $(PX4_BASE)/Images)

baremetal: 
ifeq ($(GOALS),)
	make PX4_TARGET_OS=$@ $(GOALS)
else
	export PX4_TARGET_OS=$@
endif

#
# Unittest targets. Builds and runs the host-level
# unit tests.
.PHONY: tests
tests:	
	$(Q) (mkdir -p $(PX4_BASE)/unittests/build && cd $(PX4_BASE)/unittests/build && cmake .. && $(MAKE) unittests)

.PHONY: format check_format
check_format:
	$(Q) (./Tools/check_code_style.sh | sort -n)

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything
# that's generated leaving only files that are in source control.
#
.PHONY:	clean
clean:
	@echo > /dev/null
	$(Q) $(RMDIR) $(BUILD_DIR)*.build
	$(Q) $(RMDIR) $(PX4_VERSIONING_DIR)
	$(Q) $(REMOVE) $(IMAGE_DIR)*.px4

.PHONY:	distclean
distclean: clean
	@echo > /dev/null
	$(Q) $(REMOVE) $(ARCHIVE_DIR)*.export


ifneq ($(filter upload-jtag,$(MAKECMDGOALS)),)
ifeq ($(words $(EXPLICIT_CONFIGS)),1)
LEGACY_ELF = $(BUILD_DIR)$(EXPLICIT_CONFIGS).build/firmware.elf
endif
endif

.PHONY:	upload-jtag
upload-jtag:		all flash-both

.PHONY:	flash
flash:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase $(LEGACY_ELF)" -c "reset run" -c shutdown

.PHONY: flash-bootloader
flash-bootloader:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase px4flow_bl.elf" -c "reset run" -c shutdown

.PHONY:	flash-both
flash-both:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase $(LEGACY_ELF)" -c "reset run" -c init -c "reset halt" -c "flash write_image erase px4flow_bl.elf" -c "reset run" -c shutdown

# FOR GDB
.PHONY:	flash-both-no-shutdown
flash-both-no-shutdown:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase $(LEGACY_ELF)" -c "reset run" -c init -c "reset halt" -c "flash write_image erase px4flow_bl.elf" -c "reset run"

# serial port defaults by operating system.
SYSTYPE			 = $(shell uname)
ifeq ($(SYSTYPE),Darwin)
SERIAL_PORTS		?= "/dev/tty.usbmodemPX1,/dev/tty.usbmodemPX2,/dev/tty.usbmodemPX3,/dev/tty.usbmodemPX4,/dev/tty.usbmodem1,/dev/tty.usbmodem2,/dev/tty.usbmodem3,/dev/tty.usbmodem4"
endif
ifeq ($(SYSTYPE),Linux)
SERIAL_PORTS		?= "/dev/ttyACM5,/dev/ttyACM4,/dev/ttyACM3,/dev/ttyACM2,/dev/ttyACM1,/dev/ttyACM0"
endif
ifeq ($(SERIAL_PORTS),)
SERIAL_PORTS		 = "\\\\.\\COM32,\\\\.\\COM31,\\\\.\\COM30,\\\\.\\COM29,\\\\.\\COM28,\\\\.\\COM27,\\\\.\\COM26,\\\\.\\COM25,\\\\.\\COM24,\\\\.\\COM23,\\\\.\\COM22,\\\\.\\COM21,\\\\.\\COM20,\\\\.\\COM19,\\\\.\\COM18,\\\\.\\COM17,\\\\.\\COM16,\\\\.\\COM15,\\\\.\\COM14,\\\\.\\COM13,\\\\.\\COM12,\\\\.\\COM11,\\\\.\\COM10,\\\\.\\COM9,\\\\.\\COM8,\\\\.\\COM7,\\\\.\\COM6,\\\\.\\COM5,\\\\.\\COM4,\\\\.\\COM3,\\\\.\\COM2,\\\\.\\COM1,\\\\.\\COM0"
endif

ifneq ($(filter upload-usb,$(MAKECMDGOALS)),)
ifeq ($(words $(EXPLICIT_CONFIGS)),1)
LEGACY_PX4 = $(IMAGE_DIR)$(EXPLICIT_CONFIGS).px4
endif
endif

.PHONY:	 upload-usb
upload-usb: archives all
	@echo Attempting to flash $(notdir $(LEGACY_PX4)) PX4FLOW board via USB
	@python -u Tools/px_uploader.py $(LEGACY_PX4) --baud 921600 --port $(SERIAL_PORTS)


#
# Print some help text
#
.PHONY: help
help:
	@$(ECHO) ""
	@$(ECHO) " PX4 firmware builder"
	@$(ECHO) " ===================="
	@$(ECHO) ""
	@$(ECHO) "  Available targets:"
	@$(ECHO) "  ------------------"
	@$(ECHO) ""
ifeq ($(PX4_TARGET_OS),baremetal)
	@$(ECHO) "  archives"
	@$(ECHO) "    Build the Bare Metal archives that are used by the firmware build."
	@$(ECHO) ""
endif
	@$(ECHO) "  all"
	@$(ECHO) "    Build all firmware configs: $(CONFIGS)"
	@$(ECHO) "    A limited set of configs can be built with CONFIGS=<list-of-configs>"
	@$(ECHO) ""
	@for config in $(CONFIGS); do \
		$(ECHO) "  $$config"; \
		$(ECHO) "    Build just the $$config firmware configuration."; \
		$(ECHO) ""; \
	done
	@$(ECHO) "  clean"
	@$(ECHO) "    Remove all firmware build pieces."
	@$(ECHO) ""
ifeq ($(PX4_TARGET_OS),baremetal)
	@$(ECHO) "  distclean"
	@$(ECHO) "    Remove all compilation products, including any dsdl."
	@$(ECHO) ""
	@$(ECHO) "  upload"
	@$(ECHO) "    When exactly one config is being built, add this target to upload the"
	@$(ECHO) "    firmware to the board when the build is complete. Not supported for"
	@$(ECHO) "    all configurations."
	@$(ECHO) ""
	@$(ECHO) "  upload-usb"
	@$(ECHO) "    Uploads the $(notdir $(LEGACY_PX4)) imange firmware to the board when"
	@$(ECHO) "    the build is complete. Not supported for all configurations."
	@$(ECHO) ""
endif
	@$(ECHO) "  testbuild"
	@$(ECHO) "    Perform a complete clean build of the entire tree."
	@$(ECHO) ""
	@$(ECHO) "  Common options:"
	@$(ECHO) "  ---------------"
	@$(ECHO) ""
	@$(ECHO) "  V=1"
	@$(ECHO) "    If V is set, more verbose output is printed during the build. This can"
	@$(ECHO) "    help when diagnosing issues with the build or toolchain."
	@$(ECHO) ""
ifeq ($(PX4_TARGET_OS),baremetal)
	@$(ECHO) "  To see help for a specifix target use 'make <target> help' where target is"
	@$(ECHO) "  one of: "
	@$(ECHO) "     baremetal"
	@$(ECHO) ""
endif

