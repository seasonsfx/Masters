#include <assert.h>

#include <QString>
#include <QApplication>
#include <QPluginLoader>
#include <QDebug>
#include <QDir>

#include "pluginmanager.h"

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
    activeEditPlugin = NULL;
    activeVizPlugin = NULL;
}

PluginManager::~PluginManager(){
    for(auto plugin: editPlugins){
        delete plugin;
    }
}

void PluginManager::loadPlugins(){

    qApp->addLibraryPath(qApp->applicationDirPath());

    QDir pluginsDir(qApp->applicationDirPath());
    bool succ = false;
    if(!succ)
        succ = pluginsDir.cd("plugins");
    if(!succ)
        succ = pluginsDir.cd("../plugins");
    if(!succ)
        succ = pluginsDir.cd("../lib/plugins");
    if(!succ)
        succ = pluginsDir.cd("/usr/lib/cloudclean/plugins");
    if(!succ)
        qDebug("Plugins directory not found!");

    QStringList pluginfilters("*." + DLLExtension());
    pluginsDir.setNameFilters(pluginfilters);

    foreach (QString fileName, pluginsDir.entryList(QDir::Files)){
        QString absfilepath = pluginsDir.absoluteFilePath(fileName);
        QPluginLoader loader;
        //loader.setLoadHints ( QLibrary::ResolveAllSymbolsHint );
        loader.setFileName(absfilepath);
        bool loaded = loader.load();
        if(!loaded){
            qDebug() << "Could not load: " << absfilepath;
            qDebug() << "ERROR: " << loader.errorString();

            QFile test_file(absfilepath);
            if(test_file.exists()){
                qDebug() << "File exists!";
            }
            else{
                qDebug() << "File does not exists!";
            }

            continue;
        }

        QObject *plugin = loader.instance();

        if (!loader.isLoaded()){
            qDebug() << "Could not instantiate: " << absfilepath;
            qDebug() << "ERROR: " << loader.errorString();
            continue;
        }

        if(fileName.contains("edit_")){

            EditPluginInterface * editPlugin = qobject_cast<EditPluginInterface *>(plugin);
            if(editPlugin){
                qDebug() << "Plugin loaded: " << loader.fileName();
                editPlugins.push_back(editPlugin);

                foreach(QAction* editAction, editPlugin->actions())
                    editActionList.push_back(editAction);

            }
        }

        if(fileName.contains("viz_")){

            VizPluginInterface * vizPlugin = qobject_cast<VizPluginInterface *>(plugin);
            if(vizPlugin){
                qDebug("Viz plugin loaded!\n");
                vizPlugins.push_back(vizPlugin);

                foreach(QAction* vizAction, vizPlugin->actions())
                    vizActionList.push_back(vizAction);

            }
        }

    }
}
