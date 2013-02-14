#include "cloudlist.h"
#include <QTextStream>

CloudList::CloudList(QObject *parent)
    : QAbstractListModel(parent) {
}

int CloudList::rowCount(const QModelIndex &) const {
    return clouds_.size();
}

QVariant CloudList::data(const QModelIndex & index, int role) const {
    int row = index.row();
    int col = index.column();

    if (col == 1) {
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
    clouds_.push_back(pc);
    emit cloudUpdate(clouds_.size()-1);
    return pc;
}

std::shared_ptr<PointCloud> CloudList::addCloud(const char* filename) {
    std::shared_ptr<PointCloud> pc(new PointCloud());
    pc->load_ptx(filename);
    clouds_.push_back(pc);
    emit cloudUpdate(clouds_.size()-1);
    return pc;
}

std::shared_ptr<PointCloud> CloudList::addCloud(std::shared_ptr<PointCloud> pc) {
    clouds_.push_back(pc);
    emit cloudUpdate(clouds_.size()-1);
    return pc;
}
