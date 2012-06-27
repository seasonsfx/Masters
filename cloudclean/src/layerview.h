#ifndef LAYERVIEW_H
#define LAYERVIEW_H

#include <QDockWidget>
#include <QListView>

class LayerView : public QDockWidget
{
public:
    LayerView(QWidget *parent = 0, Qt::WindowFlags flags = 0);
    QListView * layers;
private:

};

#endif // LAYERVIEW_H
