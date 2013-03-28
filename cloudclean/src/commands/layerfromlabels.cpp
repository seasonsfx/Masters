#include "layerfromlabels.h"

#include <model/layerlist.h>
#include <model/pointcloud.h>
#include <model/layer.h>

LayerFromLabels::LayerFromLabels(std::shared_ptr<std::vector<uint16_t> > labels,
                                 LayerList * ll, bool subtractive) {
    subtractive_ = subtractive;
    labels_ = labels;
    ll_ = ll;
}

QString LayerFromLabels::actionText(){
    return "New Layer";
}

void LayerFromLabels::undo(){

    // Delete layer
    if(!new_layer_.expired()){
        auto layer = new_layer_.lock();
        layer_color_ = layer->color_;
        ll_->deleteLayer(layer);
    }

    // TODO: Freelist is not updated

    // Undo subtractive
    if(subtractive)
        for(auto it : removed_from_){
            it.first()->addLabel(it.second());
        }
    }

}

void LayerFromLabels::redo(){
    std::shared_ptr<Layer> layer = ll_->addLayer();
    new_layer_ = layer;


    if(subtractive_){
        // Subtractive remove labels from layers
        for(uint16_t label : labels_) {
            LayerSet & ls = ll_->layer_lookup_table_[label];
            for(Layer * l : ls){
                if(l != ll_->default_layer_.get()){
                    removed_from_[l].push_back(label);
                    l->removeLabel(label);
                }
            }
        }

    }

    for(uint16_t label : labels_){
        layer->addLabel(label);
    }

}

bool LayerFromLabels::mergeWith(const QUndoCommand *other){
    return false;
}

int LayerFromLabels::id() const{
    return 3;
}
