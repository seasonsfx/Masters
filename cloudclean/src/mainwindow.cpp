#include "mainwindow.h"
#include "cloudmodel.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
   // Create objects
   glarea = new GLArea(this);
   layers = new LayerView(this);

   fileMenu = menuBar()->addMenu(tr("&File"));
   openFile = new QAction("Open", fileMenu);
   saveFile = new QAction("Save As", fileMenu);

   // Settings
   layers->setAllowedAreas (Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   glarea->setMinimumSize(400, 255);

   // Layout
   fileMenu->addAction(openFile);
   fileMenu->addAction(saveFile);

   addDockWidget(Qt::RightDockWidgetArea,layers);
   setCentralWidget(glarea);

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
