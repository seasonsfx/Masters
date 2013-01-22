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
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Rickert Mulder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

#include <QFileDialog>
#include <QListView>  // TODO(rick): Why do these need to be included before mainwindow?
                        // Crazy name clash? Macro?
#include "cloudclean/mainwindow.h"

#include <QMenu>
#include <QMenuBar>
#include <QApplication>

#include "cloudclean/layerview.h"
#include "cloudclean/glarea.h"
#include "cloudclean/cloudmodel.h"
#include "cloudclean/subsampledialog.h"

MainWindow::MainWindow(QWidget *parent)
     : QMainWindow(parent) {
     setWindowTitle(qApp->applicationName());

    // Create objects
    cm = new CloudModel();
    glarea = new GLArea(this, &pm, cm);
    layerView = new LayerView(this, cm);
    toolbox = new Toolbox(this);

    fileMenu = menuBar()->addMenu(tr("&File"));
    openFile = new QAction(tr("Open"), fileMenu);
    saveFile = new QAction(tr("Save As"), fileMenu);

    toolsMenu = menuBar()->addMenu(tr("&Tools"));
    vizMenu = menuBar()->addMenu(tr("&Visualization"));

    // Config
    layerView->setAllowedAreas(Qt::LeftDockWidgetArea
                                | Qt::RightDockWidgetArea);
    toolbox->setAllowedAreas(Qt::LeftDockWidgetArea
                              | Qt::RightDockWidgetArea);
    toolbox->hide();
    glarea->setMinimumSize(900, 600);

    // Layout
    fileMenu->addAction(openFile);
    fileMenu->addAction(saveFile);

    addDockWidget(Qt::RightDockWidgetArea, layerView);
    addDockWidget(Qt::RightDockWidgetArea, toolbox);
    setCentralWidget(glarea);

    // Load plugins
    pm.loadPlugins();
    for (QAction* editAction : pm.editActionList){
         toolsMenu->addAction(editAction);
         connect(editAction, SIGNAL(triggered()), this, SLOT(applyEditMode()));
    }
    for (QAction* vizAction : pm.vizActionList){
         vizMenu->addAction(vizAction);
         connect(vizAction, SIGNAL(triggered()), this, SLOT(applyVizMode()));
    }

    // Wire signals and slots
    connect(this, SIGNAL(setSettingsWidget(QWidget *)),
              toolbox, SLOT(setSettingsWidget(QWidget *)));
    connect(openFile, SIGNAL(triggered()), this, SLOT(loadScan()));
    connect(saveFile, SIGNAL(triggered()), this, SLOT(saveScan()));
    connect(&cm->layerList,
              SIGNAL(selectLayer(int)), layerView, SLOT(selectLayer(int)));
    connect(layerView, SIGNAL(updateView()),
              glarea, SLOT(updateGL()));
    connect(&cm->layerList,
              SIGNAL(updateView()), glarea, SLOT(updateGL()));
}

bool MainWindow::loadScan() {
    QString filename = QFileDialog::getOpenFileName(
                 this, tr("Open Scan"), "~", tr("PTX Files (*.ptx)"));
    if (filename.length() == 0)
        return false;

    int subsample = SubsampleDialog::getSubsample();

    if (subsample == -1)
        return false;

    qDebug() << "Filename: " << filename;

    QByteArray ba = filename.toLatin1();
    const char *c_str = ba.data();

    cm->loadFile(c_str, subsample);

    glarea->modelReloaded();
    return true;
}

bool MainWindow::loadScan(char * filename, int subsample) {
     cm->loadFile(filename, subsample);
     glarea->modelReloaded();
     return true;
}

bool MainWindow::saveScan() {
     QString filename = QFileDialog::getSaveFileName(
                     this, tr("Save Scan"), "~", tr("PTX Files (*.ptx)"));

     if (filename.length() == 0)
          return false;

     const char *ptr = filename.toAscii().data();
     cm->saveFile(ptr);
     return true;
}

void MainWindow::applyEditMode() {
     QAction *action = qobject_cast<QAction *>(sender());

     if (!cm->isLoaded()) {  // prevents crash without cloud
          action->setChecked(false);
          return;
     }

     EditPluginInterface * plugin =
             qobject_cast<EditPluginInterface *>(action->parent());

     if (pm.activeEditPlugin) {
          pm.activeEditPlugin->EndEdit(cm, glarea);
          pm.activeEditPlugin->getSettingsWidget(this)->hide();
          // Deactivate current plugin if clicked again
          if (pm.activeEditPlugin == plugin) {
                plugin->EndEdit(cm, glarea);
                pm.activeEditPlugin = NULL;
                return;
          }
          pm.activeEditPlugin = NULL;
     }

     pm.activeEditPlugin = plugin;

     QWidget * settings = pm.activeEditPlugin->getSettingsWidget(this);
     pm.activeEditPlugin->getSettingsWidget(this)->show();
     emit setSettingsWidget(settings);

     plugin->StartEdit(action, cm, glarea);
}

void MainWindow::applyVizMode() {
     QAction *action = qobject_cast<QAction *>(sender());

     if (!cm->isLoaded()) {  // prevents crash without cloud
          action->setChecked(false);
          return;
     }

     VizPluginInterface * plugin =
             qobject_cast<VizPluginInterface *>(action->parent());

     if (pm.activeVizPlugin) {
          pm.activeVizPlugin->EndViz(cm, glarea);

          // Deactivate current plugin if clicked again
          if (pm.activeVizPlugin == plugin) {
                plugin->EndViz(cm, glarea);
                pm.activeVizPlugin = NULL;
                emit setSettingsWidget(new QWidget(this));
                return;
          }
          pm.activeVizPlugin = NULL;
     }

     pm.activeVizPlugin = plugin;
     plugin->StartViz(action, cm, glarea);
}
