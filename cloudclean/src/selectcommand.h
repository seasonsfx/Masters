#ifndef SELECTCOMMAND_H
#define SELECTCOMMAND_H

#include <vector>
#include <map>
#include <memory>
#include <QUndoCommand>
#include "model/pointcloud.h"

class SelectCommand : public QUndoCommand
{
public:
    explicit SelectCommand(std::weak_ptr<PointCloud> pc,
                           std::shared_ptr<std::vector<int> > selected,
                            std::shared_ptr<std::vector<int> > deselected,
                           QUndoCommand *parent = 0);
    QString actionText();
    virtual void undo();
    virtual void redo();

signals:
    
public slots:
    
private:
    std::weak_ptr<PointCloud> pc_;
    std::shared_ptr<std::vector<int> > selected_;
    std::shared_ptr<std::vector<int> > deselected_;
};

#endif // SELECTCOMMAND_H
