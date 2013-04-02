#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <map>
#include <string>
#include <QObject>

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class QUndoStack;
class QDir;
class ActionManager;
class MainWindow;
class Core;

#include "core.h"
#include "pluginsystem/iplugin.h"

class PluginManager : public QObject{
    Q_OBJECT
 public:
    PluginManager(Core * core);
    ~PluginManager();

    bool loadPlugin(QString loc);
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
    Core * core_;
    QDir * plugins_dir_;
};

#endif // PLUGINMANAGER_H
