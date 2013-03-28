#ifndef LayerFromLabels_H
#define LayerFromLabels_H

#include <vector>
#include <memory>
#include <QUndoCommand>
#include <QColor>
#include <QString>

class LayerList;
class PointCloud;
class Layer;

class LayerFromLabels : public QUndoCommand
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
    std::vector<Layer *> removed_from_;
};

#endif // LayerFromLabels_H
