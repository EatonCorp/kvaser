#ifndef _MODULE_VERSIONING_H_
#define _MODULE_VERSIONING_H_

#if LINUX

// Module versioning
#define EXPORT_SYMTAB

#include <linux/version.h>

// Check that CONFIG_* macros are included
#if !defined(AUTOCONF_INCLUDED)
//#error linux/autoconf.h not included!
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
#include <linux/autoconf.h>
#else
#include <generated/autoconf.h>
#endif
#endif

#if defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)
#   define MODVERSIONS
#endif

#ifdef MODVERSIONS
#   if LINUX_2_6
#      include <config/modversions.h>
#   else
#      include <linux/modversions.h>
#   endif
#endif
#include <linux/module.h>

#endif // LINUX

#endif //_MODULE_VERSIONING_H_

