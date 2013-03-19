#include "cloudlist.h"
#include <QTextStream>
#include <QUndoStack>
#include <QItemSelection>
#include <QApplication>

CloudList::CloudList(QUndoStack * undostack, QObject *parent)
    : QAbstractListModel(parent) {
    mtx_ = new std::mutex();
    undostack_ = undostack;
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
    if(active_.get() == nullptr)
        active_ = pc;
    endInsertRows();
    mtx_->unlock();

    connect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SIGNAL(updated()));
    connect(pc->ed_.get(), SIGNAL(labelUpdate()), this, SIGNAL(updated()));
    connect(pc->ed_.get(), SIGNAL(transformed()), this, SIGNAL(updated()));

    emit cloudUpdate(pc);
    return pc;
}

void CloudList::removeCloud(){
    int idx = sender()->property("cloud_id").toInt();
    removeCloud(idx);
}

void CloudList::removeCloud(int idx){
    std::shared_ptr<PointCloud> pc = clouds_[idx];
    if(pc == active_)
        active_.reset();

    emit deletingCloud(pc);

    disconnect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SIGNAL(updated()));
    disconnect(pc->ed_.get(), SIGNAL(labelUpdate()), this, SIGNAL(updated()));
    disconnect(pc->ed_.get(), SIGNAL(transformed()), this, SIGNAL(updated()));


    beginRemoveRows(QModelIndex(), idx, idx);
    clouds_.erase(clouds_.begin()+idx);
    endRemoveRows();
    emit updated();
}

void CloudList::selectionChanged(const QItemSelection &sel,
                                     const QItemSelection &des) {
    selection_.clear();
    for (QModelIndex s : sel.indexes()) {
        selection_.push_back(s.row());
    }
    emit changedSelection(selection_);

    if(selection_.size() != 0)
        active_ = clouds_[selection_[0]];
        emit updatedActive(active_);
}

std::shared_ptr<PointCloud> CloudList::loadFile(QString filename){
    std::shared_ptr<PointCloud> pc;
    pc.reset(new PointCloud());
    pc->ed_->moveToThread(QApplication::instance()->thread());

    connect(pc->ed_.get(), SIGNAL(progress(int)), this, SIGNAL(progressUpdate(int)));
    pc->load_ptx(filename.toLocal8Bit().constData());
    emit progressUpdate(0);
    disconnect(pc->ed_.get(), SIGNAL(progress(int)), this, SIGNAL(progressUpdate(int)));

    emit startNonDetJob();
    addCloud(pc);
    emit endNonDetJob();

    return pc;
}

void CloudList::deselectAllPoints(){
    for(std::shared_ptr<PointCloud> & cloud : clouds_){
        for(PointFlags & flag : cloud->flags_){
            if(uint8_t(PointFlags::selected) & uint8_t(flag))
                flag = PointFlags(uint8_t(flag) & ~uint8_t(PointFlags::selected));
        }
        // TODO(Rickert): Update with vector
        cloud->ed_->emitflagUpdate();
    }
}
