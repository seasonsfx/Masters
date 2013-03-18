#ifndef CLOUDLISTVIEW_H
#define CLOUDLISTVIEW_H

#include <memory>
#include <QDockWidget>
#include <QItemSelection>
#include "model/layerlist.h"
#include "model/cloudlist.h"

namespace Ui {
class CloudListView;
}

class CloudListView : public QDockWidget
{
    Q_OBJECT
    
 public:
    explicit CloudListView(std::shared_ptr<LayerList> ll,
                           std::shared_ptr<CloudList> cl,
                           QWidget *parent = 0);
    ~CloudListView();

 private slots:
   void contextMenu(const QPoint &pos);

 public slots:
   void loadFile();

 private:
    std::shared_ptr<LayerList> ll_;
    std::shared_ptr<CloudList> cl_;
    Ui::CloudListView *ui_;
};

#endif // CLOUDLISTVIEW_H
