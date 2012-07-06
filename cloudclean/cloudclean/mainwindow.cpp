#include "mainwindow.h"
#include "cloudmodel.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{


   // Create objects
   glarea = new GLArea(this);
   layers = new LayerView(this);
   toolbox = new Toolbox(this);

   fileMenu = menuBar()->addMenu(tr("&File"));
   openFile = new QAction("Open", fileMenu);
   saveFile = new QAction("Save As", fileMenu);

   toolsMenu = menuBar()->addMenu(tr("&Tools"));

   // Config
   layers->setAllowedAreas (Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   toolbox->setAllowedAreas (Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   toolbox->hide();
   glarea->setMinimumSize(700, 500);

   // Layout
   fileMenu->addAction(openFile);
   fileMenu->addAction(saveFile);

   addDockWidget(Qt::RightDockWidgetArea,layers);
   addDockWidget(Qt::LeftDockWidgetArea,toolbox);
   setCentralWidget(glarea);

   // Load plugins
   pluginManager.loadPlugins();
   foreach(QAction* editAction, pluginManager.editActionList){
       toolsMenu->addAction(editAction);
       connect(editAction, SIGNAL(triggered()), this, SLOT(applyEditMode()));
   }

   // Wire signals and slots
   connect(openFile, SIGNAL(triggered()), this, SLOT(loadScan()));
   connect(saveFile, SIGNAL(triggered()), this, SLOT(saveScan()));
   connect(&CloudModel::Instance()->layerList, SIGNAL(selectLayer(int)), layers, SLOT(selectLayer(int)));
   connect(layers, SIGNAL(updateView()), glarea, SLOT(updateGL()));

}

bool MainWindow::loadScan(){
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Scan"), "~", tr("PTX Files (*.ptx)"));

    if(filename.length() == 0)
        return false;

    const char *ptr = filename.toAscii().data();
    CloudModel::Instance()->loadFile(ptr, 1);
    //reloadCloud();
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

    qDebug("Apply edit mode\n");

    QAction *action = qobject_cast<QAction *>(sender());

    if(!CloudModel::Instance()->isLoaded()) { //prevents crash without cloud
        action->setChecked(false);
        return;
    }

    EditPluginInterface * plugin = qobject_cast<EditPluginInterface *>(action->parent());

    if(glarea->activeEditPlugin){
        glarea->activeEditPlugin->EndEdit(CloudModel::Instance(), glarea);

        if(glarea->activeEditPlugin == plugin){
            glarea->activeEditPlugin = NULL;
            return;
        }
        glarea->activeEditPlugin = NULL;
    }

    glarea->activeEditPlugin = plugin;

    plugin->StartEdit(CloudModel::Instance(), glarea);

}
