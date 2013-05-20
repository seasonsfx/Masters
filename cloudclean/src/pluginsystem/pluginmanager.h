#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <QObject>
#include "pluginsystem/export.h"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class QUndoStack;
class QDir;
class ActionManager;
class MainWindow;
class Core;
class QPluginLoader;

#include "pluginsystem/core.h"
#include "pluginsystem/iplugin.h"

class PLUGINSYS_DLLSPEC PluginManager : public QObject{
    Q_OBJECT
 public:
    PluginManager(Core * core);
    ~PluginManager();

    IPlugin * findPluginByName(QString name);
    QString getFileName(IPlugin * plugin);
    IPlugin * loadPlugin(QString loc);
    bool unloadPlugin(IPlugin * plugin);
    void loadPlugins();
    void initializePlugins();

    template <typename T>
    T * findPlugin() {
        T * plugin = nullptr;
        for(IPlugin * aplugin : plugins_){
            plugin = qobject_cast<T *>(aplugin);
            if(plugin != nullptr)
                return plugin;
        }
        return nullptr;
    }

 signals:
    void endEdit();

 private:
    std::vector<IPlugin *> plugins_;
    std::vector<QPluginLoader *> plugin_loaders_;
    Core * core_;
    std::unique_ptr<QDir> plugins_dir_;
};

#endif // PLUGINMANAGER_H
