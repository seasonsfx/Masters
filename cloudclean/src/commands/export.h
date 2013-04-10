#ifndef COMMANDS_EXPORT_H
#define COMMANDS_EXPORT_H
#include <QtGlobal>
#ifdef commands_EXPORTS
#define DLLSPEC Q_DECL_EXPORT
#else
#define DLLSPEC Q_DECL_IMPORT
#endif
#endif  // COMMANDS_EXPORT_H
