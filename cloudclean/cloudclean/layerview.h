#ifndef LAYERVIEW_H
#define LAYERVIEW_H

#include <QDockWidget>
#include <QModelIndex>
#include <vector>

class CloudModel;

namespace Ui {
class LayerView;
}

class LayerView : public QDockWidget
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
    void clickedLayer(const QModelIndex & index);
    void selectLayer(int i);
    void deleteLayers();
    void mergeLayers();

signals:
    void updateView();
};

#endif // LAYERVIEW_H
