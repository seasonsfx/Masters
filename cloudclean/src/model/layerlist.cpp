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
    std::shared_ptr<Layer> layer(new Layer(layer_lookup_table_));
    return addLayer(layer);
}

std::shared_ptr<Layer> LayerList::addLayer(QString name) {
    std::shared_ptr<Layer> layer(new Layer(layer_lookup_table_));
    layer->name_ = name;
    return addLayer(layer);
}

void LayerList::deleteLayer(){
    deleteLayer(sender()->property("layer_id").toInt());
}

void LayerList::deleteLayer(std::shared_ptr<Layer> layer) {
    auto isEq = [&layer] (std::shared_ptr<Layer> layer2) {
            return layer == layer2;
    };

    auto iter = std::find_if(layers_.begin(), layers_.end(), isEq);
    size_t idx = std::distance(layers_.begin(), iter);

    if(idx != vec.size()) {
        deleteLayer(idx);
    }
}

void LayerList::deleteLayer(int idx){
    disconnect(layers_[idx].get(), SIGNAL(colorChanged()),
               this, SIGNAL(lookupTableUpdate()));
    beginRemoveRows(QModelIndex(), idx, idx);

    auto toDel = [&idx] (int i) {
        if(i == idx)
            return true;
        return false;
    };

    selection_.erase( remove_if(selection_.begin(), selection_.end(), toDel),
                      selection_.end() );

    for(int i = 0; i < selection_.size(); i++){
        if(selection_[i] > idx)
            --selection_[i];
    }

    layers_.erase(layers_.begin()+idx);

    endRemoveRows();
    emit lookupTableUpdate();
}

int16_t LayerList::newLabelId() {
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
    //layer_lookup_table_[++last_label_].insert(default_layer_.get());
    //layer->labels_.insert(last_label_);
    return last_label_;
}

const LayerSet & LayerList::getLayersForLabel(int i){
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

void LayerList::mergeSelectedLayers() {
    // Mark for deletion
    std::vector<std::shared_ptr<Layer>> merge_these;
    for(int idx: selection_) {
        merge_these.push_back(layers_[idx]);
    }

    // Note: selection may change when adding layer. Bugs?
    // Create new layer
    std::shared_ptr<Layer> layer = addLayer();
    layer->setName("Merged layer");

    for(int idx: selection_){
        // copy labels associated with layer
        std::shared_ptr<Layer> & l = layers_[idx];

        for(uint8_t idx: l->labels_){
            layer->addLabel(idx);
        }
    }

    // Delete marked labels
    for(std::shared_ptr<Layer> dl : merge_these){
        for(int idx = 0; idx < layers_.size(); idx++){
            if(layers_[idx] == dl){
                deleteLayer(idx);
                break;
            }
        }
    }
}

void LayerList::intersectSelectedLayers(){
    if(selection_.size() < 2){
        qDebug() << "Yeah... no. Select at least 2";
        return;
    }
    // First selected label
    std::set<uint16_t> & labels = layers_.at(selection_[0])->labels_;
    int intersection_count = 0;

    // Find intersecting labels
    std::vector<int> intersecting_labels;

    // For every label index in the first layer
    for(uint8_t label : labels){
        intersection_count = 0;
        // For every other selected layer
        for(int i = 1; i < selection_.size(); i++){
            int layer_idx = selection_[i];
            // For every label in the orther layer
            for(uint8_t qlabel : layers_[layer_idx]->labels_){
                if(label == qlabel){
                    intersection_count++;
                    break;
                }
            }
        }
        if(intersection_count == selection_.size()-1) {
            intersecting_labels.push_back(label);
        }
    }

    if(intersecting_labels.size() != 0) {
        // Remove from source
        for(uint8_t label : intersecting_labels){
            for(int layer_idx : selection_){
                std::shared_ptr<Layer> layer = layers_[layer_idx];
                layer->removeLabel(label);
            }
        }

        // Create new layer
        std::shared_ptr<Layer> layer = addLayer();
        layer->setName("New intersection");

        for(uint8_t label : intersecting_labels){
            layer->addLabel(label);
        }
    }
    else {
        qDebug() << "no interception";
    }


}

void LayerList::selectionChanged(const QItemSelection &sel,
                                     const QItemSelection &des) {

    // Labda to remove selection
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
