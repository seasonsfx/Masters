#include "newlayer.h"

#include <model/layerlist.h>
#include <model/pointcloud.h>
#include <model/layer.h>

NewLayer::NewLayer(std::shared_ptr<PointCloud> pc,
                                 std::shared_ptr<std::vector<int> > idxs,
                                 LayerList * ll) {
    pc_ = pc;
    idxs_ = idxs;
    ll_ = ll;
    applied_once_ = false;
}

QString NewLayer::actionText(){
    return "New Layer";
}

void NewLayer::undo(){
    // Change labels back
    for(int idx : *idxs_){
        pc_->labels_[idx] = new_to_old[pc_->labels_[idx]];
    }

    // Delete layer
    if(!new_layer_.expired()){
        auto layer = new_layer_.lock();
        layer_color_ = layer->color_;
        ll_->deleteLayer(layer);
    }
}

uint16_t NewLayer::getNewLabel(uint16_t old, std::shared_ptr<Layer> layer) {
    auto new_label_it = old_to_new.find(old);

    bool unknown_mapping = new_label_it == old_to_new.cend();

    if (unknown_mapping) {
        // Create a new label
        uint16_t new_label = ll_->newLabelId();
        layer->addLabel(new_label);

        ll_->copyLayerSet(old, new_label);

        // Set cache
        old_to_new[old] = new_label;
        new_to_old[new_label] = old;

        return new_label;
    }
    return old_to_new[old];
}

void NewLayer::redo(){
    std::shared_ptr<Layer> layer = ll_->addLayer();
    new_layer_ = layer;
    if(new_to_old.size() != 0)
        layer->setColor(layer_color_);


    // Relabel
    for(int idx : *idxs_){
        pc_->labels_[idx] = getNewLabel(pc_->labels_[idx], layer);
    }

    if(applied_once_) {
        // Add labels to layers
        for(auto it: new_to_old) {
            layer->addLabel(it.first);
        }
    }
    applied_once_ = true;

    pc_->ed_->emitlabelUpdate();
    pc_->ed_->emitflagUpdate();

    // TODO(Rickert) : Update color lookup buffer
}

bool NewLayer::mergeWith(const QUndoCommand *other){
    return false;
}

int NewLayer::id() const{
    return 2;
}
