#include "cloudlistview.h"
#include "ui_cloudlistview.h"
#include <QMenu>
#include <QDebug>
#include <QFileDialog>
#include <QUndoStack>
#include <thread>
#include <functional>
#include "commands/select.h"

CloudListView::CloudListView(QUndoStack *us, LayerList * ll,
                             CloudList * cl, QWidget *parent)
    : QDockWidget(parent),
    ui_(new Ui::CloudListView) {
    ll_ = ll;
    cl_ = cl;
    us_ = us;
    ui_->setupUi(this);
    ui_->tableView->setModel(cl_);
    ui_->tableView->setSelectionMode(QAbstractItemView::SingleSelection);

    connect(ui_->tableView->selectionModel(),
            SIGNAL(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)),
            cl,
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));

    connect(ui_->tableView, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

    ui_->tableView->setContextMenuPolicy(Qt::CustomContextMenu);

}

CloudListView::~CloudListView() {
    delete ui_;
}


void CloudListView::contextMenu(const QPoint &pos) {
    QMenu menu;
    QModelIndex cell = ui_->tableView->indexAt(pos);

    if(cell.isValid()){
        int row = cell.row();
        EventDispatcher * ed = cl_->clouds_[row]->ed_.get();

        menu.addAction("Reset orientation", ed,
                        SLOT(resetOrientation()));

        QAction del("Delete", 0);
        del.setProperty("cloud_id", row);
        connect(&del, SIGNAL(triggered()), cl_, SLOT(removeCloud()));
        menu.addAction(&del);

        menu.exec(ui_->tableView->mapToGlobal(pos));
    }
}

void CloudListView::deselectAllPoints(){
    us_->beginMacro("Deselect All");
    for(std::shared_ptr<PointCloud> cloud : cl_->clouds_){
        std::shared_ptr<std::vector<int> > indices;
        indices.reset(new std::vector<int>());

        std::shared_ptr<std::vector<int> > empty;
        empty.reset(new std::vector<int>());

        for(int idx = 0; idx < cloud->flags_.size(); idx++){
            PointFlags & flag =  cloud->flags_[idx];
            if(uint8_t(PointFlags::selected) & uint8_t(flag))
                indices->push_back(idx);
        }

        us_->push(new Select(cloud, empty, indices));
    }
    us_->endMacro();
}
