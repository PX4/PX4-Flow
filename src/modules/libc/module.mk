############################################################################
#
#   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
#   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
#           David Sidrane <david)s5@nscdg.com>
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
############################################################################

CLIB_INC = $(PX4_MODULE_SRC)/modules/libc/
INCLUDE_DIRS += $(CLIB_INC) $(CLIB_INC)syslog 
 
$(info INCLUDE_DIRS=$(INCLUDE_DIRS))
DEFAULT_VISIBILITY=default

# Main

STRINGSRCS =  string/lib_strncmp.c      \
							string/lib_strcmp.c       \
      				string/lib_memcpy.c       \
     					string/lib_memset.c       \
      				string/lib_strcpy.c       \
      				string/lib_strncpy.c      \
      				string/lib_strlen.c       \
      				string/lib_strncat.c      \
      				string/lib_skipspace.c    \
      				string/lib_isbasedigit.c  \
      				
STDIOSRCS =   stdio/lib_printf.c         \

STDLIBSRCS=   stdlib/lib_strtol.c        \
							stdlib/lib_strtoul.c       \
      				stdlib/lib_checkbase.c    \


SRCS = init.c \
			 $(STRINGSRCS) \
			 $(STDIOSRCS)		\
			 $(STDLIBSRCS)