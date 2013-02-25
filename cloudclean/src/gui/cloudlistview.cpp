#include "cloudlistview.h"
#include "ui_cloudlistview.h"

CloudListView::CloudListView(std::shared_ptr<LayerList> ll,
                             std::shared_ptr<CloudList> cl, QWidget *parent)
    : QDockWidget(parent),
    ui_(new Ui::CloudListView) {
    ll_ = ll;
    cl_ = cl;
    ui_->setupUi(this);
    ui_->tableView->setModel(cl_.get());
    ui_->tableView->setSelectionMode(QAbstractItemView::SingleSelection);

    connect(ui_->tableView->selectionModel(),
            SIGNAL(selectionChanged(const QItemSelection &,
                                 const QItemSelection &)), this,
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));

}

CloudListView::~CloudListView() {
    delete ui_;
}

void CloudListView::selectionChanged(const QItemSelection &sel,
                                     const QItemSelection &des) {
    for (QModelIndex s : sel.indexes()) {
        std::shared_ptr<PointCloud> cloud = cl_->clouds_[s.row()];
        emit cloudSelected(cloud);
    }
}
