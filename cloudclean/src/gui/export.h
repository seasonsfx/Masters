#ifndef GUI_EXPORT_H
#define GUI_EXPORT_H
#include <QtGlobal>
#ifdef gui_EXPORTS
#define DLLSPEC Q_DECL_EXPORT
#else
#define DLLSPEC Q_DECL_IMPORT
#endif
#endif  // GUI_EXPORT_H
