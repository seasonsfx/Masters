#ifndef LAYERVIEW_H
#define LAYERVIEW_H

#include <QDockWidget>
#include <QModelIndex>
#include <vector>
#include <QItemSelection>
#include <QAbstractItemView>

#include "cloudclean_global.h"

class CloudModel;

namespace Ui {
class LayerView;
}

class DLLSPEC LayerView : public QDockWidget
{
    Q_OBJECT
    
public:
    explicit LayerView(QWidget *parent = 0);
    ~LayerView();
    
private:
    CloudModel * cm;
    Ui::LayerView *ui;
    std::vector<int> getSelection();

public slots:
    void setSelectionMode(QAbstractItemView::SelectionMode mode);
    void selectionChanged(const QItemSelection & sel, const QItemSelection & des);
    void selectLayer(int i);
    void deleteLayers();
    void mergeLayers();

signals:
    void updateView();
};

#endif // LAYERVIEW_H
