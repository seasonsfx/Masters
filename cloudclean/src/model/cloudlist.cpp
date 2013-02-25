#include "cloudlist.h"
#include <QTextStream>

CloudList::CloudList(QObject *parent)
    : QAbstractListModel(parent) {
    mtx_ = new std::mutex();
}

CloudList::~CloudList(){
    delete mtx_;
}

int CloudList::rowCount(const QModelIndex &) const {
    return clouds_.size();
}

QVariant CloudList::data(const QModelIndex & index, int role) const {
    int row = index.row();
    int col = index.column();

    if (col == 0) {
        switch (role) {
            case Qt::DisplayRole:
            {
                QString re;
                QTextStream(&re) << "Cloud " << row;
                return re;
            }
        }
    }
    return QVariant();
}

std::shared_ptr<PointCloud> CloudList::addCloud() {
    std::shared_ptr<PointCloud> pc(new PointCloud());
    return addCloud(pc);
}

std::shared_ptr<PointCloud> CloudList::addCloud(const char* filename) {
    std::shared_ptr<PointCloud> pc(new PointCloud());
    pc->load_ptx(filename);
    return addCloud(pc);
}

std::shared_ptr<PointCloud> CloudList::addCloud(std::shared_ptr<PointCloud> pc) {
    mtx_->lock();
    beginInsertRows(QModelIndex(), clouds_.size(), clouds_.size());
    clouds_.push_back(pc);
    endInsertRows();
    mtx_->unlock();

    // TODO(rickert): emove connection when deleted
    connect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SIGNAL(updated()));
    connect(pc->ed_.get(), SIGNAL(labelUpdate()), this, SIGNAL(updated()));

    emit cloudUpdate(pc);
    return pc;
}
