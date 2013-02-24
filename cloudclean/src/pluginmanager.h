#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <map>
#include <string>
#include <QObject>
#include "plugins/plugininterfaces.h"

class PluginManager : public QObject{
    Q_OBJECT
 public:
    PluginManager();
    ~PluginManager();
    void loadPlugins();
 signals:
    void flushTest();

 private:
    std::map<QString, DataSourceIFace *> data_plugins_;
    //std::vector<
};

#endif // PLUGINMANAGER_H
