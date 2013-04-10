#ifndef PLUGINSYSTEM_EXPORT_H
#define PLUGINSYSTEM_EXPORT_H
#include <QtGlobal>
#ifdef pluginsystem_EXPORTS
#define DLLSPEC Q_DECL_EXPORT
#else
#define DLLSPEC Q_DECL_IMPORT
#endif
#endif  // PLUGINSYSTEM_EXPORT_H
