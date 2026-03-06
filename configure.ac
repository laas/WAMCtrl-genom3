dnl Autoconf file for building wamctrl codels library.
dnl
dnl Copyright (c) 2015-2024 LAAS/CNRS
dnl All rights reserved.
dnl
dnl Redistribution and use  in source  and binary  forms,  with or without
dnl modification, are permitted provided that the following conditions are
dnl met:
dnl
dnl   1. Redistributions of  source  code must retain the  above copyright
dnl      notice and this list of conditions.
dnl   2. Redistributions in binary form must reproduce the above copyright
dnl      notice and  this list of  conditions in the  documentation and/or
dnl      other materials provided with the distribution.
dnl
dnl					Johanata Brayan on Tue Mar 05 2026
dnl

AC_PREREQ(2.59)

AC_INIT([wamctrl-genom3],[2.4],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acwamctrl.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CC
AC_PROG_CXX


dnl Require GNU make
AC_CACHE_CHECK([for GNU make], [ac_cv_path_MAKE],
  [AC_PATH_PROGS_FEATURE_CHECK([MAKE], [make gmake],
    [case `$ac_path_MAKE --version 2>/dev/null` in
       *GNU*) ac_cv_path_MAKE=$ac_path_MAKE; ac_path_MAKE_found=:;;
     esac],
    [AC_MSG_ERROR([could not find GNU make])])])
AC_SUBST([MAKE], [$ac_cv_path_MAKE])


dnl External packages
AC_SUBST(requires_SPEC, ['m4_normalize([
  openrobots2-idl >= 2.0,
  genom3 >= 2.99.41
])'])
AC_SUBST(codels_requires_SPEC, ['m4_normalize([
  eigen3,
  proxsuite >= 0.6.4,
  pinocchio,
  urdfdom
])'])

PKG_CHECK_MODULES(requires, [$requires_SPEC])
PKG_CHECK_MODULES(codels_requires, [$codels_requires_SPEC])

AC_PATH_PROG(GENOM3, [genom3], [no])
if test "$GENOM3" = "no"; then
  AC_MSG_ERROR([genom3 tool not found], 2)
fi

dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [wamctrl.gen])

dnl Doc
AM_MISSING_PROG([ASCIIDOCTOR], [asciidoctor])

dnl Output
AC_CONFIG_FILES([
	wamctrl-genom3.pc
	wamctrl-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
