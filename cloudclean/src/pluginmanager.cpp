/*
 * Software License Agreement (BSD License)
 *
 *  CloudClean
 *  Copyright (c) 2013, Rickert Mulder
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rickert Mulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "pluginmanager.h"
#include <cassert>
#include <QString>
#include <QApplication>
#include <QPluginLoader>
#include <QDebug>
#include <QDir>

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

PluginManager::PluginManager() {

}

PluginManager::~PluginManager() {

}

void PluginManager::loadPlugins() {
    // Add locations to load dynamic libraries from
    qApp->addLibraryPath(qApp->applicationDirPath());

    // Look for plugin directory
    QDir pluginsDir(qApp->applicationDirPath());
    bool succ = false;
    if (!succ)
        succ = pluginsDir.cd("plugins");
    if (!succ)
        succ = pluginsDir.cd("../plugins");
    if (!succ)
        succ = pluginsDir.cd("../lib");
    if (!succ)
        succ = pluginsDir.cd("../lib/plugins");
    if (!succ)
        succ = pluginsDir.cd("/usr/lib/cloudclean/plugins");
    if (!succ){
        qDebug("Plugins directory not found!");
        return;
    }

    qDebug() << "Dir: " << pluginsDir;

    QStringList pluginfilters("*." + DLLExtension());
    pluginsDir.setNameFilters(pluginfilters);

    for (QString fileName : pluginsDir.entryList(QDir::Files)) {
        // Attempt to load library
        QString absfilepath = pluginsDir.absoluteFilePath(fileName);
        QPluginLoader loader;
        loader.setFileName(absfilepath);
        bool loaded = loader.load();
        if (!loaded) {
            qDebug() << "ERROR: " << loader.errorString();
            qDebug() << "Could not load: " << absfilepath;
            continue;
        }

        // Load success, get instance
        QObject *plugin = loader.instance();

        if (!loader.isLoaded()) {
            qDebug() << "ERROR: " << loader.errorString();
            qDebug() << "Could not instantiate: " << absfilepath;
            continue;
        }

        if (fileName.contains("data_")) {
            DataSourceIFace * dataPlugin
                    = qobject_cast<DataSourceIFace *>(plugin);
            if (dataPlugin) {
                //DataSourceIFace * ds = dataPluginFactory->getInstance();

                qDebug() << "Plugin loaded: " << loader.fileName();
                connect(this, SIGNAL(flushTest()), dataPlugin, SLOT(flushCache()));
                emit flushTest();
                qDebug() << "Flush test done";
            }
        }

    }
}
