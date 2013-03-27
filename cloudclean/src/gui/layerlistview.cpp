#include "layerlistview.h"
#include "ui_layerlistview.h"
#include <QDebug>
#include <QColorDialog>
#include <QMenu>

#include "newlayercommand.h"

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
    std::map<uint16_t, uint16_t> old_to_new;

    // So all selected points need a new label
    // A label as many layers associated with it
    // When a point gets relabled, these labels need to
    // be added to the new labels's layer set
    // The get old to new map caches these values
    // getNewLabel gets the value or calcuates a new one

    auto getNewLabel = [&old_to_new, &layer, this] (int old) {
        auto new_label_it = old_to_new.find(old);
        if (new_label_it == old_to_new.cend()) {
            // Create a new label / layer_set
            uint16_t new_label = ll_->newLabelId();
            layer->addLabel(new_label);

            ll_->copyLayerSet(old, new_label);

            // Set cache
            old_to_new[old] = new_label;
            return new_label;
        }
        return old_to_new[old];
    };

    // So it looks like a label will always have the same set of layers associated with it?


    // To make the new layer, I need to create a bunch of new labels.
    // Then I need to add existing layers to these labels in addition to the new one
    // Then in need to relabel points

    // To undo, I need to relabel the points. Remove the labels, and remove the new layer


    std::shared_ptr<std::vector<int> > points;
    std::shared_ptr<std::vector<uint16_t> > old_label;
    std::shared_ptr<std::vector<uint16_t> > new_label;


    // Undo:
    // For each point:
    //  Lookup new label
    //  For that new label, see what the old label was
    //  Now set it back to old one

    // Now delete the new label in the lookup table
    // Also delete the new layer

    std::shared_ptr<> new_layers_lookup;

    // Remap all selected points
    for(std::shared_ptr<PointCloud> & pc : cl_->clouds_){
        for(uint i = 0; i < pc->points.size(); i++){
            bool selected = uint8_t(pc->flags_[i]) & uint8_t(PointFlags::selected);

            if(selected) {
                pc->flags_[i] = PointFlags(uint8_t(pc->flags_[i]) & ~uint8_t(PointFlags::selected));
                uint16_t old_label = pc->labels_[i];
                pc->labels_[i] = getNewLabel(old_label);
            }
        }
        pc->ed_->emitlabelUpdate();
        pc->ed_->emitflagUpdate();
    }
    emit update();
    ui_->tableView->selectRow(ll_->layers_.size()-1);
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

        QAction merge("Merge", 0);
        connect(&merge, SIGNAL(triggered()), ll_.get(),
                SLOT(mergeSelectedLayers()));
        menu.addAction(&merge);

        QAction inter("Intersect", 0);
        connect(&inter, SIGNAL(triggered()), ll_.get(),
                SLOT(intersectSelectedLayers()));
        menu.addAction(&inter);

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
