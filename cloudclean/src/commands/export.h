#ifndef COMMANDS_EXPORT_H
#define COMMANDS_EXPORT_H
#include <QtGlobal>
#ifdef commands_EXPORTS
#define COMMAND_DLLSPEC Q_DECL_EXPORT
#else
#define COMMAND_DLLSPEC Q_DECL_IMPORT
#endif
#endif
