#include "pluginmanager.h"
#include <QString>
#include <QApplication>
#include <QPluginLoader>
#include <QDebug>

static QString DLLExtension() {
#if defined(Q_OS_WIN)
  return QString("dll");
#elif defined(Q_OS_MAC)
  return QString("dylib");
#else
  return QString("so");
#endif
  assert(0 && "Unknown Operative System. Please Define the appropriate dynamic library extension");
  return QString();
}

PluginManager::PluginManager()
{

}

PluginManager::~PluginManager(){
    for(auto plugin: editPlugins){
        delete plugin;
    }
}

void PluginManager::loadPlugins(){
    qApp->addLibraryPath(qApp->applicationDirPath());
    QDir pluginsDir(qApp->applicationDirPath());
    pluginsDir.cd("../plugins");
    qApp->addLibraryPath(qApp->applicationDirPath());

    QStringList pluginfilters("*." + DLLExtension());
    pluginsDir.setNameFilters(pluginfilters);

    foreach (QString fileName, pluginsDir.entryList(QDir::Files)){
        QString absfilepath = pluginsDir.absoluteFilePath(fileName);
        QPluginLoader loader;
        loader.setFileName(absfilepath);
        loader.load();
        qDebug() << loader.fileName();
        QObject *plugin = loader.instance();
        if (!loader.isLoaded()){
            qDebug() << loader.errorString();
            continue;
        }

        EditPluginInterface * editPlugin = qobject_cast<EditPluginInterface *>(plugin);
        if(editPlugin){
            qDebug("Plugin loaded!\n");
            fflush(stdout);
            editPlugins.push_back(editPlugin);

            foreach(QAction* editAction, editPlugin->actions())
                editActionList.push_back(editAction);

        }
    }
}
