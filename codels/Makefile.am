#
# Copyright (c) 2015-2024 LAAS/CNRS
# All rights reserved.
#
# Redistribution and use  in source  and binary  forms,  with or without
# modification, are permitted provided that the following conditions are
# met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
#					Johanata Brayan on Tue Mar 05 2026
#
lib_LTLIBRARIES =

# wamctrl codels library
lib_LTLIBRARIES += libwamctrl_codels.la

libwamctrl_codels_la_SOURCES  =	wamctrl_c_types.h
libwamctrl_codels_la_SOURCES +=	wamctrl_codels.c
libwamctrl_codels_la_SOURCES +=	wamctrl_main_codels.c

libwamctrl_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libwamctrl_codels_la_LIBADD   = $(requires_LIBS)
libwamctrl_codels_la_CPPFLAGS+= $(codels_requires_CFLAGS)
libwamctrl_codels_la_LIBADD  += $(codels_requires_LIBS)
libwamctrl_codels_la_LIBADD  += -lwamctrl
libwamctrl_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl mappings
BUILT_SOURCES=	wamctrl_c_types.h
CLEANFILES=	${BUILT_SOURCES}

wamctrl_c_types.h: ${top_srcdir}/wamctrl.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/wamctrl_c_types.h.d
