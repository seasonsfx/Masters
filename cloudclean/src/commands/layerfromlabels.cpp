#include "layerfromlabels.h"

#include <model/layerlist.h>
#include <model/pointcloud.h>
#include <model/layer.h>

LayerFromLabels::LayerFromLabels(boost::shared_ptr<std::vector<uint16_t> > labels,
                                 LayerList * ll, QString layer_name, bool subtractive) {
    subtractive_ = subtractive;
    labels_ = labels;
    ll_ = ll;
    layer_name_ = layer_name;
}

QString LayerFromLabels::actionText(){
    return "New Layer";
}

void LayerFromLabels::undo(){

    // Delete layer
    if(!new_layer_.expired()){
        auto layer = new_layer_.lock();
        ll_->deleteLayer(layer);
    }

    // TODO: Freelist is not updated. Feelist is bs! For now

    // Undo subtractive
    if(subtractive_) {
        for(auto it : removed_from_){
            for(uint16_t label : it.second) {
                ll_->layer_id_map_[it.first]->addLabel(label);
            }
        }
    }

}

void LayerFromLabels::redo(){
    boost::shared_ptr<Layer> layer = ll_->addLayer();
    layer->setName(layer_name_);
    new_layer_ = layer;

    if(subtractive_){
        // Subtractive remove labels from layers
        for(uint16_t label : *labels_) {
            const LayerSet & ls = ll_->getLayersForLabel(label);
            for(Layer * l : ls){
                if(l != ll_->getDefaultLayer().get()){
                    removed_from_[l->getId()].push_back(label);
                    l->removeLabel(label);
                }
            }
        }

    }

    for(uint16_t label : *labels_){
        layer->addLabel(label);
    }

}

bool LayerFromLabels::mergeWith(const QUndoCommand *other){
    return false;
}

int LayerFromLabels::id() const{
    return 4;
}
