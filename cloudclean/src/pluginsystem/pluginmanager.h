#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <map>
#include <string>
#include <QObject>

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

#include <gui/camera.h>
#include <model/cloudlist.h>
#include <model/layerlist.h>

#include "pluginsystem/iplugin.h"
#include "pluginsystem/plugininterfaces.h"

class PluginManager : public QObject{
    Q_OBJECT
 public:
    PluginManager();
    ~PluginManager();

    void loadPlugins();
    void initializePlugins(CloudList * cl, LayerList * ll);

 signals:
    void paint2d();

    void end2dEdit();
    void end3dEdit();

    void plugin3dPaint(Eigen::Affine3f, Eigen::Affine3f);
    bool plugin3dDoubleClickE(QMouseEvent * event);
    bool plugin3dMouseMoveE(QMouseEvent * event);
    bool plugin3dMousePressE(QMouseEvent * event);
    bool plugin3dMouseReleaseE(QMouseEvent * event);
    bool plugin3dWheelE(QWheelEvent * event);
    bool plugin3dKeyPressE(QKeyEvent * event);

 private:
    std::vector<IPlugin *> plugins_;

};

#endif // PLUGINMANAGER_H
