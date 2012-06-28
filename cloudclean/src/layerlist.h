#ifndef LAYERLIST_H
#define LAYERLIST_H

#include <QAbstractListModel>
#include "layer.h"
#include <vector>

class LayerList : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit LayerList(QObject *parent = 0);
    int rowCount ( const QModelIndex & parent = QModelIndex() ) const;
    int columnCount ( const QModelIndex & parent) const;
    QVariant data ( const QModelIndex & index, int role = Qt::DisplayRole ) const;
    void newLayer ();

    void reset();
    std::vector<Layer> layers;
    void activateLayer(int i);
private:

signals:
    void selectLayer(int i);

public slots:
    void deleteLayers(std::vector<int> indices);
    void mergeLayers(std::vector<int> indices);
};

#endif // LAYERLIST_H
