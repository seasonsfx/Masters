#include "layerlist.h"
#include <QTextStream>

LayerList::LayerList(QObject *parent) : QAbstractListModel(parent) {
    last_label_ = 0;
    default_layer_.reset(new Layer());
    default_layer_->name_ = "Default";
    default_layer_->setColor(QColor(255, 255, 255));
    layer_lookup_table_[0].insert(default_layer_);
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
    connect(layer.get(), SIGNAL(colorChanged()),
            this, SIGNAL(lookupTableUpdate()));
    emit layerUpdate(layer); // TODO(Rickert): Consider replacing with existing signal
    return layer;
}

std::shared_ptr<Layer> LayerList::addLayer() {
    std::shared_ptr<Layer> layer(new Layer());
    return addLayer(layer);
}

std::shared_ptr<Layer> LayerList::addLayer(QString name) {
    std::shared_ptr<Layer> layer(new Layer());
    layer->name_ = name;
    return addLayer(layer);
}

void LayerList::deleteLayer(){
    deleteLayer(sender()->property("layer_id").toInt());
}

void LayerList::deleteLayer(int idx){
    disconnect(layers_.at(idx).get(), SIGNAL(colorChanged()),
               this, SIGNAL(lookupTableUpdate()));
    beginRemoveRows(QModelIndex(), idx, idx);

    layers_.erase(layers_.begin()+idx);

    // Clean up weak pointers
    for(uint i = 0; i < this->last_label_+1; i++) {
        LayerSet & layerset = layer_lookup_table_[i];
        for(const std::weak_ptr<Layer> & wl : layerset) {
            if(wl.expired()){
                const_cast<std::weak_ptr<Layer> &>(wl) = this->default_layer_;
            }
        }
    }

    endRemoveRows();
    emit lookupTableUpdate();
}

int16_t LayerList::genLabelId(std::shared_ptr<Layer> layer) {
    layer_lookup_table_[++last_label_].insert(layer);
    return last_label_;
}


void LayerList::selectionChanged(const QItemSelection &sel,
                                     const QItemSelection &des) {

    auto toDel = [&des] (int i) {
        for(QModelIndex d : des.indexes()){
            if(i == d.row())
                return true;
        }
        return false;
    };

    selection_.erase( remove_if(selection_.begin(), selection_.end(), toDel),
                      selection_.end() );

    for (QModelIndex s : sel.indexes()) {
        selection_.push_back(s.row());
    }

    emit changedSelection(selection_);
    emit lookupTableUpdate();
}
