#include "mainwindow.h"
#include "cloudmodel.h"
#include "subsampledialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{

   // Create objects
   cm = CloudModel::Instance();
   glarea = new GLArea(this, &pm, cm);
   layerView = new LayerView(this);
   toolbox = new Toolbox(this);

   fileMenu = menuBar()->addMenu(tr("&File"));
   openFile = new QAction("Open", fileMenu);
   saveFile = new QAction("Save As", fileMenu);

   toolsMenu = menuBar()->addMenu(tr("&Tools"));
   vizMenu = menuBar()->addMenu(tr("&Visualization"));

   // Config
   layerView->setAllowedAreas (Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   toolbox->setAllowedAreas (Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   toolbox->hide();
   glarea->setMinimumSize(900, 600);

   // Layout
   fileMenu->addAction(openFile);
   fileMenu->addAction(saveFile);

   addDockWidget(Qt::RightDockWidgetArea,layerView);
   addDockWidget(Qt::RightDockWidgetArea,toolbox);
   setCentralWidget(glarea);

   // Load plugins
   pm.loadPlugins();
   foreach(QAction* editAction, pm.editActionList){
       toolsMenu->addAction(editAction);
       connect(editAction, SIGNAL(triggered()), this, SLOT(applyEditMode()));
   }
   foreach(QAction* vizAction, pm.vizActionList){
       vizMenu->addAction(vizAction);
       connect(vizAction, SIGNAL(triggered()), this, SLOT(applyVizMode()));
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

    int subsample = SubsampleDialog::getSubsample();

    if(subsample == -1)
        return false;

    const char *ptr = filename.toAscii().data();
    char* f = new char[filename.length()];
    strcpy(f,ptr); // invalid read of size 1

    CloudModel::Instance()->loadFile(f, subsample);
    delete[] f; // Mismatched free() / delete / delete []
    glarea->modelReloaded();
    return true;
}

bool MainWindow::loadScan(char * filename, int subsample){
    CloudModel::Instance()->loadFile(filename, subsample);
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

    if(pm.activeEditPlugin){
        pm.activeEditPlugin->EndEdit(CloudModel::Instance(), glarea);

        // Deactivate current plugin if clicked again
        if(pm.activeEditPlugin == plugin){
            plugin->EndEdit(CloudModel::Instance(), glarea);
            pm.activeEditPlugin = NULL;
            emit setSettingsWidget(new QWidget(this));
            return;
        }
        pm.activeEditPlugin = NULL;
    }

    pm.activeEditPlugin = plugin;

    QWidget * settings = pm.activeEditPlugin->getSettingsWidget(this);
    emit setSettingsWidget(settings);

    plugin->StartEdit(action, CloudModel::Instance(), glarea);

}

void MainWindow::applyVizMode(){

    QAction *action = qobject_cast<QAction *>(sender());

    if(!CloudModel::Instance()->isLoaded()) { //prevents crash without cloud
        action->setChecked(false);
        return;
    }

    VizPluginInterface * plugin = qobject_cast<VizPluginInterface *>(action->parent());

    if(pm.activeVizPlugin){
        pm.activeVizPlugin->EndViz(CloudModel::Instance(), glarea);

        // Deactivate current plugin if clicked again
        if(pm.activeVizPlugin == plugin){
            plugin->EndViz(CloudModel::Instance(), glarea);
            pm.activeVizPlugin = NULL;
            emit setSettingsWidget(new QWidget(this));
            return;
        }
        pm.activeVizPlugin = NULL;
    }

    pm.activeVizPlugin = plugin;

    //QWidget * settings = pm.activeVizPlugin->getSettingsWidget(this);
    //emit setSettingsWidget(settings);

    plugin->StartViz(action, CloudModel::Instance(), glarea);

}
