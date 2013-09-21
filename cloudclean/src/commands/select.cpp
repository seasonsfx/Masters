#include "select.h"

#include <QDebug>

Select::Select(std::shared_ptr<PointCloud> pc,
        std::shared_ptr<std::vector<int> > selected,
        std::shared_ptr<std::vector<int> > deselected,
        std::shared_ptr<std::vector<uint16_t> > exclude_labels,
        QUndoCommand *parent)
        : QUndoCommand(parent) {
    selected_.reset(new std::vector<int>());
    deselected_.reset(new std::vector<int>());
    pc_ = pc;

    // Check that select and deselect are not already in desired state

    auto is_selected = [&pc] (int idx) {
        return bool((uint8_t)PointFlags::selected & uint8_t(pc->flags_[idx]));
    };

    auto is_excluded = [&exclude_labels, &pc] (int idx) {
        for(uint16_t elabel : *exclude_labels) {
            if(elabel == pc->labels_[idx])
                return true;
        }
        return false;
    };

    for(int idx : *deselected) {
    if(is_selected(idx) && !is_excluded(idx))
            deselected_->push_back(idx);
    }

    for(int idx : *selected) {
        if(!is_selected(idx) && !is_excluded(idx))
            selected_->push_back(idx);
    }

}

QString Select::actionText(){
    return "Selection";
}

void Select::undo(){
    if(pc_.expired())
        return;

    std::shared_ptr<PointCloud> pc = pc_.lock();

    std::shared_ptr<std::vector<int> > update;
    update.reset(new std::vector<int>());

    //TODO(Rickert): Make sure points that are already selected/deselected

    for(int idx : *deselected_) {
        PointFlags &pf = pc->flags_[idx];
        pf = PointFlags((uint8_t(PointFlags::selected)) | uint8_t(pf));
        update->push_back(idx);
    }

    for(int idx : *selected_) {
        PointFlags &pf = pc->flags_[idx];
        pf = PointFlags(~(uint8_t(PointFlags::selected)) & uint8_t(pf));
        update->push_back(idx);
    }

    pc->flagsUpdated(update);

}

void Select::redo(){
    if(pc_.expired())
        return;

    std::shared_ptr<PointCloud> pc = pc_.lock();

    std::shared_ptr<std::vector<int> > update;
    update.reset(new std::vector<int>());

    //TODO(Rickert): Make sure points that are already selected/deselected

    for(int idx : *selected_) {
        PointFlags &pf = pc->flags_[idx];
        pf = PointFlags((uint8_t(PointFlags::selected)) | uint8_t(pf));
        update->push_back(idx);
    }

    for(int idx : *deselected_) {
        PointFlags &pf = pc->flags_[idx];
        pf = PointFlags(~(uint8_t(PointFlags::selected)) & uint8_t(pf));
        update->push_back(idx);
    }

    pc->flagsUpdated(update);

}

std::shared_ptr<std::vector<int> > mergeUnique(
        std::shared_ptr<std::vector<int> > a,
        std::shared_ptr<std::vector<int> > b){

    std::copy(b->begin(), b->end(), std::back_inserter(*a));
    std::sort(a->begin(), a->end());
    auto it = std::unique(a->begin(), a->end());
    a->resize(std::distance( a->begin(), it));

    return a;
}

bool Select::mergeWith(const QUndoCommand *other){
    if (other->id() != id())
        return false;

    const Select * o = static_cast<const Select *>(other);

    if (o->pc_.lock() != pc_.lock())
        return false;

    selected_ = mergeUnique(selected_, o->selected_);
    deselected_ = mergeUnique(deselected_, o->deselected_);
}

int Select::id() const {
    return 1;
}

