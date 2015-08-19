#
#   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
# Rules and definitions related to handling the baremetal when
# building firmware.
#

#
# For non bootloader builds add the platforms modules
#

MODULES += platforms/common

#ARCHXXINCLUDES	 += -I. -isystem $(PX4_INCLUDE_DIR) -isystem $(PX4_INCLUDE_DIR)cxx

#
# Check that the Baremetal archive for the selected board is available.
#
BAREMETTAL_ARCHIVE		:= $(wildcard $(ARCHIVE_DIR)$(BOARD).$(BAREMETAL_CONFIG).export)
ifeq ($(BAREMETTAL_ARCHIVE),)
$(error The Baremetal export archive $(BOARD).$(BAREMETAL_CONFIG).export for $(BOARD) with configuration $(BAREMETAL_CONFIG) is missing from $(ARCHIVE_DIR) - try 'make archives' in $(PX4_BASE))
endif

#
# The NuttX config header should always be present in the NuttX archive, and
# if it changes, everything should be rebuilt. So, use it as the trigger to
# unpack the NuttX archive.
#
BAREMETTAL_EXPORT_DIR	 = $(WORK_DIR)baremetal-export/

#
# Are there any start up files not in the nuttx lib
#

ifneq ($(START_UP_FILES),"")
BAREMETTAL_STARTUP = $(addprefix $(BAREMETTAL_EXPORT_DIR)startup/,$(START_UP_FILES:c=o))
$(info %  BAREMETTAL_STARTUP  = $(BAREMETTAL_STARTUP))
endif

GLOBAL_DEPS		+= BAREMETAL_CONFIG_HEADER

#
# Are there any start up files not in the nuttx lib
#

#
# Use the linker script from the NuttX export
#
LDSCRIPT		+= $(BAREMETTAL_EXPORT_DIR)build/ld.script

#
# Add directories from the bartmetal to the relevant search paths
#


INCLUDE_DIRS		+=  

LIB_DIRS		+= 
LIBS				+= 
START_OBJ		+= 
BAREMETAL_LIBS	= 


LINK_DEPS		+= $(BAREMETTAL_LIBS)

.PHONY:	BAREMETAL_CONFIG_HEADER
BAREMETAL_CONFIG_HEADER: 	$(BAREMETTAL_ARCHIVE)
	@$(ECHO) %% Unpacking $(BAREMETTAL_ARCHIVE)
	$(Q) $(UNZIP_CMD) -q -o -d $(WORK_DIR) $(BAREMETTAL_ARCHIVE)
	$(Q) $(TOUCH) $@

$(LDSCRIPT): $(BAREMETAL_CONFIG_HEADER)
$(BAREMETTAL_LIBS): $(BAREMETAL_CONFIG_HEADER)
