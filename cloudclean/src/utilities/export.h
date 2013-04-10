#ifndef UTILITIES_EXPORT_H
#define UTILITIES_EXPORT_H
#include <QtGlobal>
#ifdef utilities_EXPORTS
#define DLLSPEC Q_DECL_EXPORT
#else
#define DLLSPEC Q_DECL_IMPORT
#endif
#endif  // UTILITIES_EXPORT_H
