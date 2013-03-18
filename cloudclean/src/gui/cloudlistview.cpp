#include "cloudlistview.h"
#include "ui_cloudlistview.h"
#include <QMenu>
#include <QDebug>
#include <QFileDialog>
#include <thread>
#include <functional>

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
                                    const QItemSelection &)),
            cl.get(),
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));

    connect(ui_->tableView, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

    ui_->tableView->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(ui_->loadBtn, SIGNAL(clicked()), this, SLOT(loadFile()));
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
        connect(&del, SIGNAL(triggered()), cl_.get(), SLOT(removeCloud()));
        menu.addAction(&del);

        menu.exec(ui_->tableView->mapToGlobal(pos));
    }
}

void CloudListView::loadFile(){
    QString filename = QFileDialog::getOpenFileName(
                 this, tr("Open Scan"), "~", tr("PTX Files (*.ptx)"));
    if (filename.length() == 0)
        return;

    //auto func = std::bind(CloudList::loadFile, cl_.get());
    std::thread(&CloudList::loadFile, cl_.get(), filename).detach();
}
