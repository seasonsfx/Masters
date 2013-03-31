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
    CloudListView(QUndoStack *us, LayerList * ll,
                           CloudList * cl,
                           QWidget *parent = 0);
    ~CloudListView();

 private slots:
   void contextMenu(const QPoint &pos);

 public slots:
   void loadFile();
   void deselectAllPoints();

 private:
    LayerList * ll_;
    CloudList * cl_;
    QUndoStack *us_;
    Ui::CloudListView *ui_;
};

#endif // CLOUDLISTVIEW_H
