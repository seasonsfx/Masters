#include "layerguiselect.h"

#include <model/layerlist.h>
#include <model/pointcloud.h>
#include <model/layer.h>

LayerGuiSelect::LayerGuiSelect(boost::shared_ptr<std::vector<uint16_t> > labels,
                                 LayerList * ll, bool subtractive) {
    subtractive_ = subtractive;
    labels_ = labels;
    ll_ = ll;
}

QString LayerGuiSelect::actionText(){
    return "New Layer";
}

void LayerGuiSelect::undo(){

    // Delete layer
    if(!new_layer_.expired()){
        auto layer = new_layer_.lock();
        ll_->deleteLayer(layer);
    }

    // TODO: Freelist is not updated

    // Undo subtractive
    if(subtractive_) {
        for(auto it : removed_from_){
            for(uint16_t label : it.second)
                it.first->addLabel(label);
        }
    }

}

void LayerGuiSelect::redo(){
    boost::shared_ptr<Layer> layer = ll_->addLayer();
    new_layer_ = layer;

    if(subtractive_){
        // Subtractive remove labels from layers
        for(uint16_t label : *labels_) {
            const LayerSet & ls = ll_->getLayersForLabel(label);
            for(Layer * l : ls){
                if(l != ll_->default_layer_.get()){
                    removed_from_[l].push_back(label);
                    l->removeLabel(label);
                }
            }
        }

    }

    for(uint16_t label : *labels_){
        layer->addLabel(label);
    }

}

bool LayerGuiSelect::mergeWith(const QUndoCommand *other){
    return false;
}

int LayerGuiSelect::id() const{
    return 3;
}
