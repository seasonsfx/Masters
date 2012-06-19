#include "layerlist.h"
#include <QTextStream>

LayerList::LayerList(QObject *parent) :
    QAbstractListModel(parent)
{
}

int LayerList::rowCount ( const QModelIndex & parent) const {
    printf("Called\n");
    return 1;//layers.size();
}

QVariant LayerList::data ( const QModelIndex & index, int role ) const {
    QString re;
    QTextStream(&re) << "Layer " << index.row();
    printf("Called 2\n");
    return re;
}

/*QVariant LayerList::headerData ( int section, Qt::Orientation orientation, int role ) const {
    return QVariant();
}*/
