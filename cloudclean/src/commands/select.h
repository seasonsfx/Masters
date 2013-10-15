#ifndef SELECTCOMMAND_H
#define SELECTCOMMAND_H

#include <vector>
#include <map>
#include <memory>
#include <QUndoCommand>
#include <boost/weak_ptr.hpp>
#include "model/pointcloud.h"
#include "commands/export.h"

class COMMAND_API Select : public QUndoCommand
{
public:
    explicit Select(boost::shared_ptr<PointCloud> pc,
                           boost::shared_ptr<std::vector<int> > selected,
                           boost::shared_ptr<std::vector<int> > deselected = boost::shared_ptr<std::vector<int> >(new std::vector<int>()),
                           boost::shared_ptr<std::vector<uint16_t> > exclude_labels = boost::shared_ptr<std::vector<uint16_t> >(new std::vector<uint16_t>()),
                           QUndoCommand *parent = 0);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;
    
private:
    boost::weak_ptr<PointCloud> pc_;
    boost::shared_ptr<std::vector<int> > selected_;
    boost::shared_ptr<std::vector<int> > deselected_;
};

#endif // SELECTCOMMAND_H
