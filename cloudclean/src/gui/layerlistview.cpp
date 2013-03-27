#include "layerlistview.h"
#include "ui_layerlistview.h"
#include <QDebug>
#include <QColorDialog>
#include <QMenu>

#include "newlayercommand.h"
#include "selectcommand.h"

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

// TODO(Rickert): selections should also be updated via a vector

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

        cl_->undostack_->push(new NewLayerCommand(pc, idxs, ll_.get()));
        cl_->undostack_->push(new SelectCommand(pc, empty, idxs));

    }
    cl_->undostack_->endMacro();
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
