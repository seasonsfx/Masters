#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H
#include <QDir>
#include "../common/interfaces.h"
#include <vector>

class PluginManager
{
public:
    PluginManager();
    void loadPlugins();
    ~PluginManager();

    QVector<QAction *> editActionList;
    std::vector<EditPluginInterface *> editPlugins;
};

#endif // PLUGINMANAGER_H
