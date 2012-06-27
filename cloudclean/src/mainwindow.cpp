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

   // Layout
   fileMenu->addAction(openFile);
   fileMenu->addAction(saveFile);

   addDockWidget(Qt::RightDockWidgetArea,layers);
   setCentralWidget(glarea);

   // Wire signals and slots
   connect(openFile, SIGNAL(triggered()), this, SLOT(loadScan()));
   connect(saveFile, SIGNAL(triggered()), this, SLOT(saveScan()));
   connect(layers->layers, SIGNAL(clicked(const QModelIndex &)), this, SLOT(clickedLayer(const QModelIndex &)));
   //connect(this, SIGNAL(reloadCloud()), glarea, SLOT(reloadCloud()));
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

void MainWindow::clickedLayer(const QModelIndex & index){
    CloudModel * cm = CloudModel::Instance();
    int idx = index.row();
    cm->layerList.layers[idx].toggleActive();
    emit glarea->updateGL();
}

void MainWindow::selectLayer(int i){
    //CloudModel * cm = CloudModel::Instance();
    //cm->layerList.layers[i].active = true;
    //ui->layerList->setSelection ( QRect(i, i, 0, 1), QItemSelectionModel::Select);
}
