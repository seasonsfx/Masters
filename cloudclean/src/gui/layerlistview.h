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
    explicit LayerListView(std::shared_ptr<LayerList> ll,
                           std::shared_ptr<CloudList> cl, QWidget *parent = 0);
    ~LayerListView();

 private slots:
    void selectionToLayer();
    void contextMenu(const QPoint &pos);
    void setColor();

 private:
    std::shared_ptr<LayerList> ll_;
    std::shared_ptr<CloudList> cl_;
    Ui::LayerListView *ui_;
};

#endif // LAYERLISTVIEW_H
