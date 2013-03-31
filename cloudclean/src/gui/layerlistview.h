#ifndef LAYERLISTVIEW_H
#define LAYERLISTVIEW_H

#include <memory>
#include <QDockWidget>
#include <QItemSelection>
#include "model/layerlist.h"
#include "model/cloudlist.h"

namespace Ui {
class LayerListView;
}

class LayerListView : public QDockWidget
{
    Q_OBJECT
    
 public:
    explicit LayerListView(QUndoStack *us, LayerList * ll,
                           CloudList * cl, QWidget *parent = 0);
    ~LayerListView();

 public slots:
    void selectLayer(std::shared_ptr<Layer> layer);

 private slots:
    void selectionToLayer();
    void intersectSelectedLayers();
    void mergeSelectedLayers();
    void contextMenu(const QPoint &pos);
    void setColor();

 private:
    LayerList * ll_;
    CloudList * cl_;
    QUndoStack * us_;
    Ui::LayerListView *ui_;
};

#endif // LAYERLISTVIEW_H
