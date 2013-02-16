#include "layerlist.h"
#include <QTextStream>

LayerList::LayerList(QObject *parent) : QAbstractListModel(parent) {
    last_label_id_ = -1;
    mtx_ = new std::mutex();
}

LayerList::~LayerList(){
    delete mtx_;
}

int LayerList::rowCount(const QModelIndex &) const {
    return layers_.size();
}

QVariant LayerList::data(const QModelIndex & index, int role) const {
    int row = index.row();
    int col = index.column();

    if (col == 0) {
        switch (role) {
        case Qt::DisplayRole:
        {
            QString re;
            QTextStream(&re) << layers_[row]->name_;
            return re;
        }
        case Qt::DecorationRole:
            return layers_[row]->color_;
        }
    }
    return QVariant();
}

std::shared_ptr<Layer> LayerList::addLayer(std::shared_ptr<Layer> layer){
    mtx_->lock();
    beginInsertRows(QModelIndex(), layers_.size(), layers_.size());
    layers_.push_back(layer);
    endInsertRows();
    mtx_->unlock();
    emit layerUpdate(layer); // TODO(Rickert): Consider replacing with existing signal
    return layer;
}

std::shared_ptr<Layer> LayerList::addLayer() {
    std::shared_ptr<Layer> layer(new Layer());
    return addLayer(layer);
}

std::shared_ptr<Layer> LayerList::addLayer(QString name, QColor color) {
    std::shared_ptr<Layer> layer(new Layer());
    layer->name_ = name;
    layer->color_ = color;
    return addLayer(layer);
}

int16_t LayerList::genLabelId(std::shared_ptr<Layer> layer) {
    layer_lookup_table_[++last_label_id_] = layer;
    //emit lookupTableUpdate();
    return last_label_id_;
}
