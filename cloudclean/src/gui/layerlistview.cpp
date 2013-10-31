#include "layerlistview.h"
#include "ui_layerlistview.h"
#include <QDebug>
#include <QColorDialog>
#include <QMenu>
#include <boost/make_shared.hpp>

#include "commands/newlayer.h"
#include "commands/select.h"
#include "commands/layerfromlabels.h"
#include "commands/layerdelete.h"

LayerListView::LayerListView(QUndoStack * us, LayerList * ll,
                             CloudList * cl, QWidget *parent) :
        QDockWidget(parent), ui_(new Ui::LayerListView) {
    ll_ = ll;
    cl_ = cl;
    us_ = us;
    ui_->setupUi(this);
    ui_->tableView->setModel(ll_);

    connect(ui_->tableView->selectionModel(),
            SIGNAL(selectionChanged(const QItemSelection &,
                                 const QItemSelection &)), ll_,
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));

    ui_->tableView->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui_->tableView, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));
    ui_->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui_->tableView->setColumnWidth(0, 30);
    ui_->tableView->horizontalHeader()->setStretchLastSection(true);
    ui_->tableView->setStyleSheet(
        "QTableView::indicator:unchecked {image: url(:/eye_closed.png);}"
        "QTableView::indicator:checked {image: url(:/eye_open.png);}"
    );
}

LayerListView::~LayerListView() {
    delete ui_;
}

void LayerListView::selectLayer(boost::shared_ptr<Layer> layer) {

}

// TODO(Rickert): selections should also be updated via a vector


// This function is here because i do not want to put the layerlist in the
// cloudlist
void LayerListView::selectionToLayer(){
    us_->beginMacro("Layer from selection");
    // Remap all selected points
    for(boost::shared_ptr<PointCloud> & pc : cl_->clouds_){
        std::vector<boost::shared_ptr<std::vector<int> >> selections = pc->getSelections();

        for(size_t sel_idx = 0; sel_idx < selections.size(); sel_idx++){
            boost::shared_ptr<std::vector<int> > selection = selections[sel_idx];
            if(selection->size() == 0)
                continue;

            qDebug() << "Selection " << sel_idx << " has " << selection->size() << "selections";

            us_->push(new NewLayer(pc, selection, ll_));
            us_->push(new Select(pc, nullptr, selection, 1 << sel_idx));
        }
    }

    us_->endMacro();
    emit update();
    ui_->tableView->selectRow(ll_->layers_.size()-1);
}

void LayerListView::intersectSelectedLayers(){
    if(ll_->selection_.size() < 2){
        qDebug() << "Select at least 2 layers";
        return;
    }


    // First selected label
    std::set<uint16_t> & labels = ll_->selection_[0].lock()->labels_;
    uint intersection_count = 0;

    // Find intersecting labels
    boost::shared_ptr<std::vector<uint16_t> > intersecting_labels;
    intersecting_labels.reset(new std::vector<uint16_t>());

    // For every label index in the first layer
    for(uint8_t label : labels){
        intersection_count = 0;
        // For every other selected layer
        for(uint i = 1; i < ll_->selection_.size(); i++){
            boost::shared_ptr<Layer> layer = ll_->selection_[i].lock();
            // For every label in the orther layer
            for(uint8_t qlabel : layer->labels_){
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

    us_->beginMacro("Layer intersection");
    us_->push(new LayerFromLabels(intersecting_labels, ll_, true));
    us_->endMacro();

    ui_->tableView->selectRow(ll_->layers_.size()-1);

}

void LayerListView::mergeSelectedLayers() {
    // Mark for deletion
    std::vector<boost::shared_ptr<Layer> > merge_these;
    for(boost::weak_ptr<Layer> l: ll_->selection_) {
        merge_these.push_back(l.lock());
    }


    boost::shared_ptr<std::vector<uint16_t> > labels;
    labels.reset(new std::vector<uint16_t>());


    for(boost::weak_ptr<Layer> l: ll_->selection_){
        boost::shared_ptr<Layer> layer = l.lock();
        for(int label : layer->labels_){
            labels->push_back(label);
        }
    }

    // Get rid of duplicates
    std::sort(labels->begin(), labels->end());
    labels->erase( unique(labels->begin(), labels->end()), labels->end());

    us_->beginMacro("Merge layers");
    us_->push(new LayerFromLabels(labels, ll_, true));


    // Delete marked labels
    for(boost::shared_ptr<Layer> dl : merge_these) {
        us_->push(new LayerDelete(dl, ll_));
    }


    us_->endMacro();
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
        connect(&del, SIGNAL(triggered()), ll_, SLOT(deleteLayer()));
        menu.addAction(&del);

        QAction merge("Merge", 0);
        connect(&merge, SIGNAL(triggered()), this,
                SLOT(mergeSelectedLayers()));
        menu.addAction(&merge);

        QAction inter("Intersect", 0);
        connect(&inter, SIGNAL(triggered()), this,
                SLOT(intersectSelectedLayers()));
        menu.addAction(&inter);

        QAction select_layer("Select points", 0);
        connect(&select_layer, &QAction::triggered, [=] () {

            std::set<uint16_t> selected_labels;
            for(boost::weak_ptr<Layer> wl : ll_->selection_) {
                boost::shared_ptr<Layer> l = wl.lock();
                for(uint16_t label  : l->getLabelSet()) {
                    selected_labels.insert(label);
                }
            }

            us_->beginMacro("Select layer");
            for(boost::shared_ptr<PointCloud> pc : cl_->clouds_){
                boost::shared_ptr<std::vector<int>> points = boost::make_shared<std::vector<int>>();
                for(uint idx = 0; idx < pc->size(); ++idx) {
                    for(uint16_t slabel : selected_labels) {
                        if(slabel == pc->labels_[idx]){
                            points->push_back(idx);
                            break;
                        }
                    }
                }

                us_->push(new Select(pc, points));

            }
            us_->endMacro();
        });
        menu.addAction(&select_layer);

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
