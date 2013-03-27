#ifndef NEWLAYERCOMMAND_H
#define NEWLAYERCOMMAND_H

#include <vector>
#include <memory>
#include <QUndoCommand>
#include <QColor>
#include <QString>

class LayerList;
class PointCloud;

class NewLayerCommand : public QUndoCommand
{
 public:
    NewLayerCommand(std::shared_ptr<PointCloud> pc,
                    std::shared_ptr<std::vector<int> > idxs,
                    LayerList * ll);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;

 private:
    std::map<uint16_t, uint16_t> old_to_new;
    std::map<uint16_t, uint16_t> new_to_old;

    std::shared_ptr<PointCloud> pc_;
    std::shared_ptr<std::vector<int> > idxs_;
    LayerList * ll_;

    std::weak_ptr<Layer> new_layer_;
    QColor layer_color_;

};

#endif // NEWLAYERCOMMAND_H
