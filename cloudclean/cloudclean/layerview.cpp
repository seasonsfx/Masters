#include "layerview.h"
#include "ui_layerview.h"
#include "cloudmodel.h"
#include <QItemSelectionModel>

LayerView::LayerView(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::LayerView)
{
    cm = CloudModel::Instance();

    ui->setupUi(this);
    ui->listView->horizontalHeader()->hide();
    ui->listView->verticalHeader()->hide();
    ui->listView->horizontalHeader()->setStretchLastSection(true);
    ui->listView->setGridStyle(Qt::NoPen);
    ui->listView->setModel(&CloudModel::Instance()->layerList);
    ui->listView->setSelectionMode(QAbstractItemView::MultiSelection);
    ui->listView->setColumnWidth (0, 30);
    ui->listView->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->selection_mode_combo->insertItem(0, "Create new layer");
    ui->selection_mode_combo->insertItem(1, "Add to active layer");
    ui->selection_mode_combo->insertItem(2, "Remove points");

    connect(ui->listView, SIGNAL(pressed(const QModelIndex &)), this, SLOT(pressed(const QModelIndex &)));
    connect(ui->delete_button, SIGNAL(pressed()), this, SLOT(deleteLayers()));
    connect(ui->merge_button, SIGNAL(pressed()), this, SLOT(mergeLayers()));
    connect(ui->selection_mode_combo, SIGNAL(currentIndexChanged (int)), &cm->layerList, SLOT(selectModeChanged(int)));
}

LayerView::~LayerView()
{
    delete ui;
}

void LayerView::pressed(const QModelIndex & index){
    int col = index.column();
    int row = index.row();

    if(col == 1 || col == 0){
        cm->layerList.layers[row].toggleActive();
        emit updateView();
    }
}

void LayerView::selectLayer(int i){
    cm->layerList.layers[i].active = true;
    QItemSelectionModel * sm = ui->listView->selectionModel();
    QModelIndex mi = sm->model()->index(i, 0, QModelIndex());
    sm->select(mi, QItemSelectionModel::Select);

    mi = sm->model()->index(i, 1, QModelIndex());
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
    cm->layerList.deleteLayers(getSelection());
    emit updateView();
}

void LayerView::mergeLayers(){
    cm->layerList.mergeLayers(getSelection());
    emit updateView();
}
