#ifndef GUI_EXPORT_H
#define GUI_EXPORT_H
#include <QtGlobal>
#ifdef gui_EXPORTS
#define GUI_DLLSPEC Q_DECL_EXPORT
#else
#define GUI_DLLSPEC Q_DECL_IMPORT
#endif
#endif
