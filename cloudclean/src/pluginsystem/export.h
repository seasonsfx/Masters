#ifndef PLUGINSYSTEM_EXPORT_H
#define PLUGINSYSTEM_EXPORT_H
#include <QtGlobal>
#ifdef pluginsystem_EXPORTS
#define PLUGINSYS_DLLSPEC Q_DECL_EXPORT
#else
#define PLUGINSYS_DLLSPEC Q_DECL_IMPORT
#endif
#endif
