#ifndef LayerDelete_H
#define LayerDelete_H

#include <vector>
#include <map>
#include <memory>
#include <QUndoCommand>
#include <QColor>
#include <QString>

class LayerList;
class PointCloud;
class Layer;

class LayerDelete : public QUndoCommand {
 public:
    LayerDelete(std::shared_ptr<Layer> layer, LayerList * ll);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;

 private:
    std::vector<uint16_t> labels_;
    QColor col_;
    QString name_;
    LayerList * ll_;
    uint id_;
    std::weak_ptr<Layer> layer_;
};

#endif // LayerFromLabels_H
