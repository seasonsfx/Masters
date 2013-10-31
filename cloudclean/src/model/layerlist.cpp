#include "layerlist.h"
#include <QTextStream>
#include <QDebug>

LayerList::LayerList(QObject *parent) : QAbstractListModel(parent) {
    last_label_ = 0;
    default_layer_.reset(new Layer(layer_lookup_table_));
    default_layer_->setName("Default");
    default_layer_->setColor(QColor(255, 255, 255));
    layer_lookup_table_[0].insert(default_layer_.get());
    mtx_ = new std::mutex();
}

LayerList::~LayerList(){
    delete mtx_;
}

Qt::ItemFlags LayerList::flags(const QModelIndex & index) const {
    int col = index.column();

    if (col == 0) {
        return Qt::ItemIsUserCheckable | Qt::ItemIsEnabled
                | Qt::ItemIsSelectable;
    }

    return Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled;
}

int LayerList::columnCount(const QModelIndex &) const {
    return 2;
}

int LayerList::rowCount(const QModelIndex &) const {
    return layers_.size();
}

QVariant LayerList::data(const QModelIndex & index, int role) const {
    int row = index.row();
    int col = index.column();

    if (col == 1) {
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
    } else if (col == 0) {
        switch (role) {
        case Qt::CheckStateRole:
                return layers_[row]->isVisible() ? Qt::Checked : Qt::Unchecked;
        }
    }

    return QVariant();
}

bool LayerList::setData(const QModelIndex & index, const QVariant & value,
                        int role) {
    int row = index.row();
    int col = index.column();

    if (role == Qt::CheckStateRole && col == 0) {
        layers_[row]->toggleVisible();
        emit dataChanged(index, index);
        emit lookupTableUpdate();
    }
    return true;
}

boost::shared_ptr<Layer> LayerList::addLayer(boost::shared_ptr<Layer> layer){
    mtx_->lock();
    beginInsertRows(QModelIndex(), layers_.size(), layers_.size());
    layer_id_map_[layer->id_] = layer;
    layers_.push_back(layer);
    endInsertRows();
    mtx_->unlock();
    connect(layer.get(), SIGNAL(colorChanged()),
            this, SIGNAL(lookupTableUpdate()));
    emit layerUpdate(layer); // TODO(Rickert): Consider replacing with existing signal
    return layer;
}

boost::shared_ptr<Layer> LayerList::addLayerWithId(uint id) {
    boost::shared_ptr<Layer> layer(new Layer(layer_lookup_table_));
    layer->id_ = id;
    return addLayer(layer);
}

boost::shared_ptr<Layer> LayerList::addLayer() {
    boost::shared_ptr<Layer> layer(new Layer(layer_lookup_table_));
    return addLayer(layer);
}

boost::shared_ptr<Layer> LayerList::addLayer(QString name) {
    boost::shared_ptr<Layer> layer(new Layer(layer_lookup_table_));
    layer->name_ = name;
    return addLayer(layer);
}

int LayerList::getLayerIndex(boost::shared_ptr<Layer> layer) const{
    auto isEq = [&layer] (boost::shared_ptr<Layer> layer2) {
            return layer == layer2;
    };

    auto iter = std::find_if(layers_.begin(), layers_.end(), isEq);
    size_t idx = std::distance(layers_.begin(), iter);

    if(idx == layers_.size())
        return -1;

    return idx;
}


uint LayerList::getLastLabel() const {
    return last_label_;
}

const std::vector<boost::weak_ptr<Layer> > & LayerList::getSelection() const {
    return selection_;
}

boost::shared_ptr<Layer> LayerList::getDefaultLayer() const {
    return default_layer_;
}

const std::vector<boost::shared_ptr<Layer> > & LayerList::getLayers() {
    return layers_;
}


void LayerList::deleteLayer(){
    int id = sender()->property("layer_id").toInt();
    deleteLayer(id);
}

void LayerList::deleteLayer(boost::shared_ptr<Layer> layer) {
    int idx = getLayerIndex(layer);

    if(idx != -1) {
        deleteLayer(idx);
    }
}

void LayerList::deleteLayer(int idx){
    disconnect(layers_[idx].get(), SIGNAL(colorChanged()),
               this, SIGNAL(lookupTableUpdate()));
    beginRemoveRows(QModelIndex(), idx, idx);

    auto toDel = [&idx, this] (boost::weak_ptr<Layer> l) {
        if(l.expired())
            return true;
        return false;
    };

    // If selected, fix that state up
    selection_.erase( remove_if(selection_.begin(), selection_.end(), toDel),
                      selection_.end() );

    // this is a bug! if the labels go to a free list then a new layer
    // will have funky selection
    // Add labels to free list (Label should maybe do this)
    /*
    for(int label : layers_[idx]->labels_) {
        LayerSet & ls = layer_lookup_table_[label];
        bool is_free = true;
        for(Layer * l: ls) {
            if(l != this->default_layer_.get() && l !=  layers_[idx].get()){
                is_free = false;
                break;
            }
        }
        if(is_free)
            this->free_labels_.push_back(label);
    }*/

    // Do actual deleting of layer
    layer_id_map_.erase(layers_[idx]->id_);
    layers_.erase(layers_.begin()+idx);

    endRemoveRows();
    emit lookupTableUpdate();
}

int16_t LayerList::createLabelId() {
    if(last_label_ == 0xFFFF){
        qDebug() << "Whoops, we ran out of shorts";
        exit(1);
    }

    if(free_labels_.size() > 0) {
        int16_t id = free_labels_.back();
        free_labels_.pop_back();
        return id;
    }
    // Add this label to default layer
    default_layer_->addLabel(++last_label_);
    return last_label_;
}

const LayerSet & LayerList::getLayersForLabel(int i) {
    return layer_lookup_table_[i];
}

void LayerList::copyLayerSet(uint8_t source_label, uint8_t dest_label){
    LayerSet & source_layerset = layer_lookup_table_[source_label];
    LayerSet & dest_layerset = layer_lookup_table_[dest_label];

    // Add labels from source set to dest
    for(Layer * layer : source_layerset) {
        dest_layerset.insert(layer);
        layer->addLabel(dest_label);
    }
    emit lookupTableUpdate(); // TODO(Rickert): Might be inefficient with many calls to this slot
}

void LayerList::selectionChanged(const QItemSelection &sel,
                                     const QItemSelection &des) {

    // Labda to remove selection
    auto toDel = [&des, this] (boost::weak_ptr<Layer> l) {
        for(QModelIndex d : des.indexes()){
            if(l.expired())
                return true; // this should not happen, just in case take care of it
            if(l.lock() == layers_[d.row()])
                return true;
        }
        return false;
    };

    selection_.erase( remove_if(selection_.begin(), selection_.end(), toDel),
                      selection_.end() );

    for (QModelIndex s : sel.indexes()) {
        selection_.push_back(layers_[s.row()]);
    }

    emit changedSelection(selection_);
    emit lookupTableUpdate();
}
