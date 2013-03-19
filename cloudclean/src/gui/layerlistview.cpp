#include "layerlistview.h"
#include "ui_layerlistview.h"
#include <QDebug>
#include <QColorDialog>
#include <QMenu>

LayerListView::LayerListView(std::shared_ptr<LayerList> ll,
                             std::shared_ptr<CloudList> cl, QWidget *parent) :
        QDockWidget(parent), ui_(new Ui::LayerListView) {
    ll_ = ll;
    cl_ = cl;
    ui_->setupUi(this);
    ui_->tableView->setModel(ll_.get());

    connect(ui_->tableView->selectionModel(),
            SIGNAL(selectionChanged(const QItemSelection &,
                                 const QItemSelection &)), ll_.get(),
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));

    connect(ui_->saveSelectionBtn, SIGNAL(clicked()), this, SLOT(selectionToLayer()));
    ui_->tableView->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui_->tableView, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));
}

LayerListView::~LayerListView() {
    delete ui_;
}

// TODO(Rickert): selections should also be in a vector

void LayerListView::selectionToLayer(){
    // New layer
    auto layer = ll_->addLayer();

    // Map old label to new
    std::map<uint8_t, uint8_t> old_to_new;

    auto getNewLabel = [&old_to_new, &layer, this] (int old) {
        auto new_label_it = old_to_new.find(old);
        if (new_label_it == old_to_new.cend()) {
            // Create a new label set
            uint8_t new_label = ll_->genLabelId(layer);

            LayerSet & old_layerset = ll_->layer_lookup_table_[old];
            LayerSet & new_layerset = ll_->layer_lookup_table_[new_label];

            // Add labels from old set to new
            for(auto layer : old_layerset) {
                new_layerset.insert(layer);
            }

            // Set cache
            old_to_new[old] = new_label;
            return new_label;
        }
        return old_to_new[old];
    };

    // Remap all selected points
    for(std::shared_ptr<PointCloud> pc : cl_->clouds_){
        for(uint i = 0; i < pc->points.size(); i++){
            bool selected = uint8_t(pc->flags_[i]) & uint8_t(PointFlags::selected);

            if(selected) {
                uint8_t old_label = pc->labels_[i];
                pc->labels_[i] = getNewLabel(old_label);
            }
        }
        pc->ed_->emitlabelUpdate();
    }
}

void LayerListView::contextMenu(const QPoint &pos) {
    QMenu menu;
    QModelIndex cell = ui_->tableView->indexAt(pos);

    if(cell.isValid()){
        int row = cell.row();

        QAction randCol("Random color", 0);
        randCol.setProperty("layer_id", row);
        randCol.setProperty("random", true);
        connect(&randCol, SIGNAL(triggered()), this, SLOT(setColor()));
        menu.addAction(&randCol);

        QAction changeCol("Change color", 0);
        changeCol.setProperty("layer_id", row);
        changeCol.setProperty("random", false);
        connect(&changeCol, SIGNAL(triggered()), this, SLOT(setColor()));
        menu.addAction(&changeCol);

        QAction del("Delete", 0);
        del.setProperty("layer_id", row);
        connect(&del, SIGNAL(triggered()), ll_.get(), SLOT(deleteLayer()));
        menu.addAction(&del);

        QAction selectLayer("Select", 0);
        menu.addAction(&selectLayer);

        menu.exec(ui_->tableView->mapToGlobal(pos));
    }
}

void LayerListView::setColor() {
    int id = sender()->property("layer_id").toInt();
    bool random = sender()->property("random").toBool();
    Layer & l = *ll_->layers_[id];
    if(random)
        l.setRandomColor();
    else
        l.setColor(QColorDialog::getColor(l.color_));
    emit update();
}
