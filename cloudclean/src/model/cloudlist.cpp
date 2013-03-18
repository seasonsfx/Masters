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

    /*
    pc->ed_->moveToThread(QApplication::instance()->thread());

    connect(pc->ed_.get(), SIGNAL(progress(int)), progressbar_, SLOT(setValue(int)));
    pc->load_ptx(fname);
    disconnect(pc->ed_.get(), SIGNAL(progress(int)), progressbar_, SLOT(setValue(int)));

    QMetaObject::invokeMethod(progressbar_, "setRange", Q_ARG(int, 0), Q_ARG(int, 0));

    cl_->addCloud(pc);

    // make a selection
    std::vector<PointFlags> & flags = pc->flags_;
    for(uint i = 0; i < flags.size()/5; i++){
        flags[i] = PointFlags::selected;
    }

    // label the cloud
    std::vector<int16_t> & labels = pc->labels_;
    for(uint i = 0; i < labels.size(); i++){
        labels[i] = i%5;
    }

    // create layers with colors
    std::shared_ptr<Layer> layers[3];
    layers[0] = ll_->addLayer("Test1", QColor(255, 0, 0));
    layers[1] = ll_->addLayer("Test2", QColor(0, 255, 0));
    layers[2] = ll_->addLayer("Test3", QColor(0, 0, 255));

    // make five labels
    for(int i = 0; i < 5; i++)
        ll_->genLabelId(layers[i%3]);

    qRegisterMetaType<std::shared_ptr<PointCloud> >("std::shared_ptr<PointCloud>");
    QMetaObject::invokeMethod(flatview_, "setCloud", Q_ARG(std::shared_ptr<PointCloud>, pc));

    glwidget_->update();
    QMetaObject::invokeMethod(progressbar_, "setRange", Q_ARG(int, 0), Q_ARG(int, 100));
    QMetaObject::invokeMethod(progressbar_, "reset");
    qDebug() << "Loaded";
    qDebug() << "Size: " << pc->size();
    */
    return pc;
}
