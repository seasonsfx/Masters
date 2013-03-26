#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <map>
#include <string>
#include <QObject>

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class ActionManager;

#include "gui/camera.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include <model/cloudlist.h>
#include <model/layerlist.h>
#include "pluginsystem/iplugin.h"

class PluginManager : public QObject{
    Q_OBJECT
 public:
    PluginManager(GLWidget * glwidget, FlatView * flatview, CloudList * cl, LayerList * ll, ActionManager * am);
    ~PluginManager();

    void loadPlugins();
    void initializePlugins();

 signals:
    void endEdit();

 private:
    std::vector<IPlugin *> plugins_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    CloudList * cl_;
    LayerList * ll_;
    ActionManager * am_;

};

#endif // PLUGINMANAGER_H
