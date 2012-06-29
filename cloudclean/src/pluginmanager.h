#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H
#include <QDir>
#include "interfaces.h"
#include <vector>

class PluginManager
{
public:
    PluginManager();
    void loadPlugins();
    ~PluginManager();

    std::vector<EditPluginInterface *> editPlugins;
};

#endif // PLUGINMANAGER_H
