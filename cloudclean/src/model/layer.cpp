#include "model/layer.h"
#include <cstdlib>
#include <QDebug>

Layer::Layer(std::map<uint8_t, LayerSet> & layer_lookup_table)
    : layer_lookup_table_(layer_lookup_table) {
    name_ = "New Layer";
    color_.setHsl(rand()%359, rand()%55 + 200, 127);
}

Layer::~Layer(){
    for(uint8_t label : labels_){
        // Why would this fail?... think aout it
        auto it = layer_lookup_table_[label].find(this);
        if(it != layer_lookup_table_[label].end())
            layer_lookup_table_[label].erase(it);
    }
    qDebug() << "Layer " << name_ << "deleted";
}

void Layer::setColor(QColor color){
    color_ = color;
    emit colorChanged();
}

void Layer::setRandomColor(){
    color_.setHsl(rand()%359, rand()%55 + 200, 127);
    emit colorChanged();
}

void Layer::setName(QString name) {
    name_ = name;
}

void Layer::addLabel(uint8_t id) {
    layer_lookup_table_[id].insert(this);
    labels_.insert(id);
}

void Layer::removeLabel(uint8_t id) {
    auto it = layer_lookup_table_[id].find(this);
    if(it != layer_lookup_table_[id].end())
        layer_lookup_table_[id].erase(it);
}

const std::set<uint8_t> & Layer::getLabelSet(){
    return labels_;
}
