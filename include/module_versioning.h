#ifndef _MODULE_VERSIONING_H_
#define _MODULE_VERSIONING_H_

#if LINUX

// Module versioning
#define EXPORT_SYMTAB

// Check that CONFIG_* macros are included
#if !defined(AUTOCONF_INCLUDED)
//#error linux/autoconf.h not included!
#include <generated/autoconf.h>
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

