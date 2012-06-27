#ifndef LASSOSELECTPLUGIN_H
#define LASSOSELECTPLUGIN_H

#include <QtPlugin>
#include <QObject>
#include "interfaces/editplugininterface.h"

class LassoSelectPlugin : public QObject, public EditPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(EditPluginInterface)
public:
    LassoSelectPlugin();
};

#endif // LASSOSELECTPLUGIN_H
