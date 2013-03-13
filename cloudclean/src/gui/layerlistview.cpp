#include "layerlistview.h"
#include "ui_layerlistview.h"
#include <QDebug>

LayerListView::LayerListView(std::shared_ptr<LayerList> ll,
                             std::shared_ptr<CloudList> cl, QWidget *parent) :
        QDockWidget(parent), ui_(new Ui::LayerListView) {
    ll_ = ll;
    cl_ = cl;
    ui_->setupUi(this);
    ui_->tableView->setModel(ll_.get());

    connect(ui_->tableView->selectionModel(),
            SIGNAL(selectionChanged(const QItemSelection &,
                                 const QItemSelection &)), this,
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));
}

LayerListView::~LayerListView() {
    delete ui_;
}


void LayerListView::selectionChanged(const QItemSelection &sel,
                                     const QItemSelection &des) {
    qDebug() << "Selection changed";
    for (QModelIndex s : sel.indexes()) {
        std::shared_ptr<Layer> layer = ll_->layers_[s.row()];

        for(std::shared_ptr<PointCloud> pc : cl_->clouds_){
            for(uint i = 0; i < pc->flags_.size(); i++){
                // if label in layer
                uint8_t label = pc->labels_[i];
                if(ll_->layer_lookup_table_[label].lock() == layer){
                    PointFlags & pf = pc->flags_[i];
                    pf = PointFlags(uint8_t(PointFlags::selected) | uint8_t(pf));
                }
            }
        }
    }

    for (QModelIndex s : des.indexes()) {
        std::shared_ptr<Layer> layer = ll_->layers_[s.row()];

        for(std::shared_ptr<PointCloud> pc : cl_->clouds_){
            for(uint i = 0; i < pc->flags_.size(); i++){
                // if label in layer
                uint8_t label = pc->labels_[i];
                if(ll_->layer_lookup_table_[label].lock() == layer){
                    PointFlags & pf = pc->flags_[i];
                    pf = PointFlags((~uint8_t(PointFlags::selected)) & uint8_t(pf));
                }
            }
        }
    }

    // Update flags
    for(std::shared_ptr<PointCloud> pc : cl_->clouds_){
        pc->ed_->emitflagUpdate();
    }
}
