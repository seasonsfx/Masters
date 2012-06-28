#include "layerview.h"
#include "ui_layerview.h"
#include "cloudmodel.h"
#include <QItemSelectionModel>

LayerView::LayerView(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::LayerView)
{
    ui->setupUi(this);
    ui->listView->setModel(&CloudModel::Instance()->layerList);
    ui->listView->setSelectionMode(QAbstractItemView::MultiSelection);
    connect(ui->listView, SIGNAL(clicked(const QModelIndex &)), this, SLOT(clickedLayer(const QModelIndex &)));
    connect(ui->delete_button, SIGNAL(pressed()), this, SLOT(deleteLayers()));
    connect(ui->merge_button, SIGNAL(pressed()), this, SLOT(mergeLayers()));
}

LayerView::~LayerView()
{
    delete ui;
}

void LayerView::clickedLayer(const QModelIndex & index){
    int col = index.column();
    int row = index.row();
    CloudModel * cm = CloudModel::Instance();

    if(col == 0){
        cm->layerList.layers[row].toggleActive();
        emit updateView();
    }
    /*else if(col == 1){
        cm->layerList.layers[row].toggleVisible();
        emit glarea->updateGL();
    }
    */
}

void LayerView::selectLayer(int i){
    CloudModel * cm = CloudModel::Instance();
    cm->layerList.layers[i].active = true;
    QItemSelectionModel * sm = ui->listView->selectionModel();
    QModelIndex mi = sm->model()->index(i, 0, QModelIndex());
    sm->select(mi, QItemSelectionModel::Select);
}

std::vector<int> LayerView::getSelection(){
    std::vector<int> re;
    QItemSelectionModel * sm = ui->listView->selectionModel();
    QModelIndexList sl = sm->selectedRows();
    QModelIndex mi;
    foreach(mi, sl){
        re.push_back(mi.row());
    }
    return re;
}

void LayerView::deleteLayers(){
    CloudModel * cm = CloudModel::Instance();
    cm->layerList.deleteLayers(getSelection());
    emit updateView();
}

void LayerView::mergeLayers(){
    CloudModel * cm = CloudModel::Instance();
    cm->layerList.mergeLayers(getSelection());
    emit updateView();
}
