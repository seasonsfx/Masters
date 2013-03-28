#include "layerlistview.h"
#include "ui_layerlistview.h"
#include <QDebug>
#include <QColorDialog>
#include <QMenu>

#include "commands/newlayer.h"
#include "commands/select.h"
#include "commands/layerfromlabels.h"
#include "commands/layerdelete.h"

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

void LayerListView::selectLayer(std::shared_ptr<Layer> layer) {

}

// TODO(Rickert): selections should also be updated via a vector


// This function is here because i do not want to put the layerlist in the
// cloudlist
void LayerListView::selectionToLayer(){
    cl_->undostack_->beginMacro("Layer from selection");
    // Remap all selected points
    for(std::shared_ptr<PointCloud> & pc : cl_->clouds_){
        std::shared_ptr<std::vector<int> > idxs;
        idxs.reset(new std::vector<int>());

        std::shared_ptr<std::vector<int> > empty;
        empty.reset(new std::vector<int>());

        for(uint i = 0; i < pc->points.size(); i++){
            bool selected = uint8_t(pc->flags_[i]) & uint8_t(PointFlags::selected);

            if(selected)
                idxs->push_back(i);
        }

        cl_->undostack_->push(new NewLayer(pc, idxs, ll_.get()));
        cl_->undostack_->push(new Select(pc, empty, idxs));

    }
    cl_->undostack_->endMacro();
    emit update();
    ui_->tableView->selectRow(ll_->layers_.size()-1);
}

void LayerListView::intersectSelectedLayers(){
    if(ll_->selection_.size() < 2){
        qDebug() << "Select at least 2 layers";
        return;
    }


    // First selected label
    std::set<uint16_t> & labels = ll_->layers_.at(ll_->selection_[0])->labels_;
    int intersection_count = 0;

    // Find intersecting labels
    std::shared_ptr<std::vector<uint16_t> > intersecting_labels;
    intersecting_labels.reset(new std::vector<uint16_t>());

    // For every label index in the first layer
    for(uint8_t label : labels){
        intersection_count = 0;
        // For every other selected layer
        for(int i = 1; i < ll_->selection_.size(); i++){
            int layer_idx = ll_->selection_[i];
            // For every label in the orther layer
            for(uint8_t qlabel : ll_->layers_[layer_idx]->labels_){
                if(label == qlabel){
                    intersection_count++;
                    break;
                }
            }
        }
        if(intersection_count == ll_->selection_.size()-1) {
            intersecting_labels->push_back(label);
        }
    }


    if(intersecting_labels->size() == 0){
        qDebug() << "no intersection";
        return;
    }

    cl_->undostack_->beginMacro("Layer intersection");
    cl_->undostack_->push(new LayerFromLabels(intersecting_labels, ll_.get(), true));
    cl_->undostack_->endMacro();

    ui_->tableView->selectRow(ll_->layers_.size()-1);

}

void LayerListView::mergeSelectedLayers() {
    // Mark for deletion
    std::vector<std::shared_ptr<Layer> > merge_these;
    for(int idx: ll_->selection_) {
        merge_these.push_back(ll_->layers_[idx]);
    }


    std::shared_ptr<std::vector<uint16_t> > labels;
    labels.reset(new std::vector<uint16_t>());


    for(int idx: ll_->selection_){
        Layer & layer = *ll_->layers_[idx];
        for(int label : layer.labels_){
            labels->push_back(label);
        }
    }

    // Get rid of duplicates
    std::sort(labels->begin(), labels->end());
    labels->erase( unique(labels->begin(), labels->end()), labels->end());

    cl_->undostack_->beginMacro("Merge layers");
    cl_->undostack_->push(new LayerFromLabels(labels, ll_.get(), true));


    // Delete marked labels
    for(std::shared_ptr<Layer> dl : merge_these) {
        cl_->undostack_->push(new LayerDelete(dl, ll_.get()));
    }


    cl_->undostack_->endMacro();
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
        connect(&merge, SIGNAL(triggered()), this,
                SLOT(mergeSelectedLayers()));
        menu.addAction(&merge);

        QAction inter("Intersect", 0);
        connect(&inter, SIGNAL(triggered()), this,
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
