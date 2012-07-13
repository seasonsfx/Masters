#include "mainwindow.h"
#include "cloudmodel.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{


   // Create objects
   glarea = new GLArea(this);
   layerView = new LayerView(this);
   toolbox = new Toolbox(this);

   fileMenu = menuBar()->addMenu(tr("&File"));
   openFile = new QAction("Open", fileMenu);
   saveFile = new QAction("Save As", fileMenu);

   toolsMenu = menuBar()->addMenu(tr("&Tools"));

   // Config
   layerView->setAllowedAreas (Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   toolbox->setAllowedAreas (Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   toolbox->hide();
   glarea->setMinimumSize(700, 500);

   // Layout
   fileMenu->addAction(openFile);
   fileMenu->addAction(saveFile);

   addDockWidget(Qt::RightDockWidgetArea,layerView);
   addDockWidget(Qt::RightDockWidgetArea,toolbox);
   setCentralWidget(glarea);

   // Load plugins
   pluginManager.loadPlugins();
   foreach(QAction* editAction, pluginManager.editActionList){
       toolsMenu->addAction(editAction);
       connect(editAction, SIGNAL(triggered()), this, SLOT(applyEditMode()));
   }

   // Wire signals and slots
   connect(this, SIGNAL(setSettingsWidget(QWidget *)), toolbox, SLOT(setSettingsWidget(QWidget *)));
   connect(openFile, SIGNAL(triggered()), this, SLOT(loadScan()));
   connect(saveFile, SIGNAL(triggered()), this, SLOT(saveScan()));
   connect(&CloudModel::Instance()->layerList, SIGNAL(selectLayer(int)), layerView, SLOT(selectLayer(int)));
   connect(layerView, SIGNAL(updateView()), glarea, SLOT(updateGL()));
   connect(&CloudModel::Instance()->layerList, SIGNAL(updateView()), glarea, SLOT(updateGL()));

}

bool MainWindow::loadScan(){
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Scan"), "~", tr("PTX Files (*.ptx)"));

    if(filename.length() == 0)
        return false;

    const char *ptr = filename.toAscii().data();
    CloudModel::Instance()->loadFile(ptr, 1);
    glarea->modelReloaded();
    return true;
}

bool MainWindow::saveScan(){
    QString filename = QFileDialog::getSaveFileName(this, tr("Save Scan"), "~", tr("PTX Files (*.ptx)"));

    if(filename.length() == 0)
        return false;

    const char *ptr = filename.toAscii().data();
    CloudModel::Instance()->saveFile(ptr);
    return true;
}

void MainWindow::applyEditMode(){

    QAction *action = qobject_cast<QAction *>(sender());

    if(!CloudModel::Instance()->isLoaded()) { //prevents crash without cloud
        action->setChecked(false);
        return;
    }

    EditPluginInterface * plugin = qobject_cast<EditPluginInterface *>(action->parent());

    if(glarea->activeEditPlugin){
        glarea->activeEditPlugin->EndEdit(CloudModel::Instance(), glarea);

        // Deactivate current plugin if clicked again
        if(glarea->activeEditPlugin == plugin){
            plugin->EndEdit(CloudModel::Instance(), glarea);
            glarea->activeEditPlugin = NULL;
            emit setSettingsWidget(new QWidget(this));
            return;
        }
        glarea->activeEditPlugin = NULL;
    }

    glarea->activeEditPlugin = plugin;

    QWidget * settings = glarea->activeEditPlugin->getSettingsWidget(this);
    emit setSettingsWidget(settings);

    plugin->StartEdit(action, CloudModel::Instance(), glarea);

}
