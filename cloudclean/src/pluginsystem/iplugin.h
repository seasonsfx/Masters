#ifndef IPLUGIN_H
#define IPLUGIN_H

#include <QObject>
#include <QtPlugin>

class GLWidget;
class FlatView;
class CloudList;
class LayerList;
class PluginManager;
class ActionManager;

// So plugins are created with the default constructor
// This cannot take any parameters so use initialise method
// Plugins get to register its slots with signals from other plugins
// or the main app
// How do i stop two plugins from being active at once?
// When a plugin goes active it should emit a signal
// This signal should
// Plugins get to hook into 2d or 3d draw
// Plugins get to listen for mouse events in the 2d or 3d view

class IPlugin: public QObject  {
    Q_OBJECT
 public:
    virtual QString getName() = 0;
    virtual void initialize(PluginManager *pm, ActionManager * am,
                            CloudList * cl, LayerList * ll,
                            GLWidget * glwidget, FlatView * flatview) = 0;
    virtual void cleanup() = 0;
};

Q_DECLARE_INTERFACE(IPlugin, "za.co.circlingthesun.cloudclean.plugininterface/1.0")

#endif // IPLUGIN_H