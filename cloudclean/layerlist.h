#ifndef LAYERLIST_H
#define LAYERLIST_H

#include <QAbstractListModel>
#include "layer.h"

class LayerList : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit LayerList(QObject *parent = 0);
    int rowCount ( const QModelIndex & parent = QModelIndex() ) const;
    QVariant data ( const QModelIndex & index, int role = Qt::DisplayRole ) const;
    //QVariant headerData ( int section, Qt::Orientation orientation, int role = Qt::DisplayRole ) const;

private:
    std::vector<Layer> layers;

signals:
    
public slots:
    
};

#endif // LAYERLIST_H
