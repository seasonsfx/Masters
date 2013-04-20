#ifndef UTILITIES_EXPORT_H
#define UTILITIES_EXPORT_H
#include <QtGlobal>
#ifdef utilities_EXPORTS
#define UTIL_DLLSPEC Q_DECL_EXPORT
#else
#define UTIL_DLLSPEC Q_DECL_IMPORT
#endif
#endif
