#include <QFileDialog>
#include "ui_cloudclean.h"
#include "appdata.h"
#include "cloudclean.h"

CloudClean::CloudClean(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CloudClean)
{
    ui->setupUi(this);

    connect(ui->openButton, SIGNAL(pressed()), this, SLOT(loadScan()));
    connect(ui->saveButton, SIGNAL(pressed()), this, SLOT(saveScan()));

    bool scan_loaded = loadScan();
    int cloud_size_not_0 = AppData::Instance()->cloud->points.size();
    if( !( cloud_size_not_0|| scan_loaded) ){
        exit(0);
    }

    // Add GL widget to window
    glwidget = new GLWidget;
    ui->gl->addWidget(glwidget);
    ui->layerList->setModel(&AppData::Instance()->layerList);
    ui->layerList->setSelectionMode(QAbstractItemView::MultiSelection);

    connect(ui->layerList, SIGNAL(clicked(const QModelIndex &)), this, SLOT(clickedLayer(const QModelIndex &)));

    connect(this, SIGNAL(reloadCloud()), glwidget, SLOT(reloadCloud()));
}

CloudClean::~CloudClean()
{
    delete ui;
}

void CloudClean::clickedLayer(const QModelIndex & index){
    //QModelIndexList selected = ui->layerList->selectedIndexes();
    AppData * app_data = AppData::Instance();

    int idx = index.row();
    app_data->layerList.layers[idx].toggleActive();

    emit glwidget->updateGL();

    /*QModelIndex item;
    foreach (item, selected) {
    }*/

    /*
    // Deactivate all
    for(int i = 0; i < app_data->layerList.layers.size(); i++){
        app_data->layerList.layers[i].active = false;
    }

    // Activate selected
    for(int i = 0; i < sRows.size(); i++){
        int idx = sRows[i].row();
        app_data->layerList.layers[idx].active = true;
    }
*/
}

void CloudClean::selectLayer(int i){
    AppData * app_data = AppData::Instance();
    //app_data->layerList.layers[i].active = true;
    //ui->layerList->setSelection ( QRect(i, i, 0, 1), QItemSelectionModel::Select);
}

bool CloudClean::loadScan(){
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Scan"), "~", tr("PTX Files (*.ptx)"));
	
    if(filename.length() == 0)
        return false;

    const char *ptr = filename.toAscii().data();
    AppData::Instance()->loadFile(ptr, 1);
    reloadCloud();
    return true;
}

bool CloudClean::saveScan(){
    QString filename = QFileDialog::getSaveFileName(this, tr("Save Scan"), "~", tr("PTX Files (*.ptx)"));

    if(filename.length() == 0)
        return false;

    const char *ptr = filename.toAscii().data();
    AppData::Instance()->saveFile(ptr);
    return true;
}
