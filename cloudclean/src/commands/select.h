#ifndef SELECTCOMMAND_H
#define SELECTCOMMAND_H

#include <vector>
#include <map>
#include <memory>
#include <QUndoCommand>
#include "model/pointcloud.h"
#include "commands/export.h"

class DLLSPEC Select : public QUndoCommand
{
public:
    explicit Select(std::shared_ptr<PointCloud> pc,
                           std::shared_ptr<std::vector<int> > selected,
                           std::shared_ptr<std::vector<int> > deselected = std::shared_ptr<std::vector<int> >(new std::vector<int>()),
                           QUndoCommand *parent = 0);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;
    
private:
    std::weak_ptr<PointCloud> pc_;
    std::shared_ptr<std::vector<int> > selected_;
    std::shared_ptr<std::vector<int> > deselected_;
};

#endif // SELECTCOMMAND_H
