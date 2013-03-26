#include "selectcommand.h"

SelectCommand::SelectCommand(std::weak_ptr<PointCloud> pc,
        std::shared_ptr<std::vector<int> > selected,
        std::shared_ptr<std::vector<int> > deselected,
        QUndoCommand *parent)
        : QUndoCommand(parent) {
    selected_ = selected;
    deselected_ = deselected;
    pc_ = pc;
}

QString SelectCommand::actionText(){
    return "Selection";
}

void SelectCommand::undo(){
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

    pc->ed_->emitflagUpdate(update);

}

void SelectCommand::redo(){
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

    pc->ed_->emitflagUpdate(update);

}
