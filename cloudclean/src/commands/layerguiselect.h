#ifndef LayerGuiSelect_H
#define LayerGuiSelect_H

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

class COMMAND_DLLSPEC LayerGuiSelect : public QUndoCommand
{
 public:
    LayerGuiSelect(std::shared_ptr<std::vector<uint16_t> > labels,
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
    std::map<Layer *, std::vector<uint16_t> > removed_from_;
};

#endif // LayerGuiSelect_H
