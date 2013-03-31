#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <map>
#include <string>
#include <QObject>

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class QUndoStack;
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

    void loadPlugins();
    void initializePlugins();

 signals:
    void endEdit();

 private:
    std::vector<IPlugin *> plugins_;
    Core * core_;

};

#endif // PLUGINMANAGER_H
