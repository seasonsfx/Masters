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

#include "pluginsystem/pluginmanager.h"
#include <cassert>
#include <QString>
#include <QApplication>
#include <QPluginLoader>
#include <QDebug>
#include <QDir>
#include <QUndoStack>
#include <gui/mainwindow.h>
#include <model/layerlist.h>
#include <model/cloudlist.h>

#include "pluginsystem/core.h"

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

PluginManager::PluginManager(Core * core) {
    // Add locations to load dynamic libraries from
    qApp->addLibraryPath(qApp->applicationDirPath());
    core_ = core;

    // Look for plugin directory
    plugins_dir_.reset(new QDir(qApp->applicationDirPath()));
    bool succ = false;
    if (!succ)
        succ = plugins_dir_->cd("plugins");
    if (!succ)
        succ = plugins_dir_->cd("../plugins");
    if (!succ)
        succ = plugins_dir_->cd("../lib/plugins");
    if (!succ)
        succ = plugins_dir_->cd("../lib");
    if (!succ)
        succ = plugins_dir_->cd("/usr/lib/cloudclean/plugins");
    if (!succ){
		plugins_dir_.reset();
        qDebug("Plugins directory not found!");
		return;
    }

    qApp->addLibraryPath(plugins_dir_->absolutePath());

}

PluginManager::~PluginManager() {
    /*
    for(IPlugin * plugin: plugins_){
        plugin->cleanup();
        delete plugin;
    }
    for(QPluginLoader * loader: plugin_loaders_){
        loader->unload();
        delete loader;
    }
    */
}

IPlugin * PluginManager::findPluginByName(QString name) {
    for(IPlugin * plugin: plugins_) {
        if(plugin != nullptr && plugin->getName() == name)
            return plugin;
    }
    return nullptr;
}

bool PluginManager::unloadPlugin(IPlugin * plugin){
    int pos = -1;
    for(int idx = 0; idx < plugins_.size(); idx++) {
        if(plugins_[idx] == plugin) {
            pos = idx;
            break;
        }
    }

    if(pos == -1) {
        qDebug() << "Could not find plugin!";
        return false;
    }

    // Remove instance
    plugins_[pos]->cleanup();
    delete plugins_[pos];
    plugins_.erase(plugins_.begin() + pos);

    // Unload plugin
    bool unloaded = plugin_loaders_[pos]->unload();

    qDebug() << "Still loaded? " << plugin_loaders_[pos]->isLoaded();

    if(!unloaded)
        qDebug() << "Could not unload library";
    delete plugin_loaders_[pos];
    plugin_loaders_.erase(plugin_loaders_.begin() + pos);
}

QString PluginManager::getFileName(IPlugin * plugin){
    int pos = -1;
    for(int idx = 0; idx < plugins_.size(); idx++) {
        if(plugins_[idx] == plugin) {
            pos = idx;
            break;
        }
    }

    if(pos == -1) {
        qDebug() << "Could not find plugin!";
        return "";
    }


    return plugin_loaders_[pos]->fileName();
}

IPlugin * PluginManager::loadPlugin(QString loc){
    QPluginLoader * loader = new QPluginLoader();

    loader->setLoadHints(QLibrary::ExportExternalSymbolsHint|QLibrary::ResolveAllSymbolsHint);
    loader->setFileName(loc);
    bool loaded = loader->load();

    if (!loaded) {
        qDebug() << "Could not load plugin: " << loc;
        qDebug() << "ERROR: " << loader->errorString();
        delete loader;
        return nullptr;
    }

    QObject *plugin = loader->instance();

    IPlugin * iplugin = qobject_cast<IPlugin *>(plugin);
    if(!iplugin){
        qDebug() << "Not a valid plugin";
        delete loader;
        return nullptr;
    }

    plugin_loaders_.push_back(loader);
    plugins_.push_back(iplugin);
    return iplugin;
}

void PluginManager::loadPlugins() {
    if(!plugins_dir_)
		return;

    QStringList pluginfilters("*." + DLLExtension());
    plugins_dir_->setNameFilters(pluginfilters);

    for (QString fileName : plugins_dir_->entryList(QDir::Files)) {
        QString absfilepath = plugins_dir_->absoluteFilePath(fileName);
        loadPlugin(absfilepath);
    }
}

void PluginManager::initializePlugins() {
    for(IPlugin * plugin : plugins_){
        plugin->initialize(core_);
    }

    for(IPlugin * plugin : plugins_){
        plugin->initialize2(this);
    }
}


