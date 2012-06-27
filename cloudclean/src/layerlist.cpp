#include "layerlist.h"
#include <QTextStream>
#include "cloudmodel.h"

LayerList::LayerList(QObject *parent) :
    QAbstractListModel(parent)
{
}

int LayerList::rowCount ( const QModelIndex & parent) const {
    Q_UNUSED(parent);
    return layers.size();
}

QVariant LayerList::data ( const QModelIndex & index, int role ) const {

    if( role == Qt::DisplayRole ) {
        QString re;
        QTextStream(&re) << "Layer " << index.row();
        return re;
    }

    return QVariant();

}

void LayerList::newLayer (){
    int pos = layers.size();
    beginInsertRows( QModelIndex(), pos, pos );
    layers.push_back(Layer());
    endInsertRows();
}

/// Watch out, has OpenGL context been initialized
void LayerList::reset (){
    if(layers.size() < 2)
        return;
    layers.erase(layers.begin()+1, layers.end());
    Layer & layer = layers[0];

    CloudModel * app_data = CloudModel::Instance();

    layer.gl_index_buffer.allocate(app_data->cloud->size()*sizeof(int));

    for(int i = 1; i < app_data->cloud->size(); i++){
        layer.index[i] = i;
    }
    layer.copyToGPU();
}

/*QVariant LayerList::headerData ( int section, Qt::Orientation orientation, int role ) const {
    QString re;
    QTextStream(&re) << "Layer " << index.row();
    printf("Called 2\n");
    return re;
}
*/
