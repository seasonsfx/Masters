#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H
#include <QDir>
#include "../common/interfaces.h"
#include "../common/testinterface.h"
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
