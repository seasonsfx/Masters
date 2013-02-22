#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <QObject>

class PluginManager : public QObject{
    Q_OBJECT
 public:
   PluginManager();
   ~PluginManager();
   void loadPlugins();
 signals:
   void flushTest();
};

#endif // PLUGINMANAGER_H
