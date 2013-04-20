#ifndef LayerFromLabels_H
#define LayerFromLabels_H

#include <vector>
#include <map>
#include <memory>
#include <QUndoCommand>
#include <QColor>
#include <QString>
#include "commands/export.h"

class LayerList;
class PointCloud;
class Layer;

class COMMAND_DLLSPEC LayerFromLabels : public QUndoCommand
{
 public:
    LayerFromLabels(std::shared_ptr<std::vector<uint16_t> > labels,
                    LayerList * ll, bool subtractive = true);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;

 private:
    bool subtractive_;
    std::shared_ptr<std::vector<uint16_t> > labels_;
    LayerList * ll_;
    std::weak_ptr<Layer> new_layer_;
    std::map<uint, std::vector<uint16_t> > removed_from_;
};

#endif // LayerFromLabels_H
