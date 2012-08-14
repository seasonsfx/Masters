#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H
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
    EditPluginInterface* activeEditPlugin;

    QVector<QAction *> vizActionList;
    std::vector<VizPluginInterface *> vizPlugins;
    VizPluginInterface* activeVizPlugin;
};

#endif // PLUGINMANAGER_H
