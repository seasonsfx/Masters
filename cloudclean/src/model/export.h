#ifndef MODEL_EXPORT_H
#define MODEL_EXPORT_H
#include <QtGlobal>
#ifdef model_EXPORTS
#define DLLSPEC Q_DECL_EXPORT
#else
#define DLLSPEC Q_DECL_IMPORT
#endif
#endif  // MODEL_EXPORT_H
