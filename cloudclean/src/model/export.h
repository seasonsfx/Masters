#ifndef MODEL_EXPORT_H
#define MODEL_EXPORT_H
#include <QtGlobal>
#ifdef model_EXPORTS
#define MODEL_DLLSPEC Q_DECL_EXPORT
#else
#define MODEL_DLLSPEC Q_DECL_IMPORT
#endif
#endif
