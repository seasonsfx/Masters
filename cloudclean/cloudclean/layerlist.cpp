#include "layerlist.h"
#include <QTextStream>
#include "cloudmodel.h"
 #include <QColor>

LayerList::LayerList(QObject *parent) :
    QAbstractListModel(parent)
{
}

int LayerList::rowCount ( const QModelIndex & parent) const {
    Q_UNUSED(parent);
    return layers.size();
}

int LayerList::columnCount ( const QModelIndex & parent) const {
    Q_UNUSED(parent);
    return 1;
}

QVariant LayerList::data ( const QModelIndex & index, int role ) const {
    int row = index.row();

    switch(role){
    case Qt::DisplayRole:
    {
        QString re;
        QTextStream(&re) << "Layer " << row;
        return re;
    }
    case Qt::DecorationRole:
        return QColor(layers[row].colour[0]*255, layers[row].colour[1]*255, layers[row].colour[2]*255);
    //case Qt::CheckStateRole:
    //        break;//return layers[row].visible? Qt::Checked:Qt::Unchecked;
    }
    return QVariant();

}

void LayerList::newLayer (){
    int pos = layers.size();
    beginInsertRows( QModelIndex(), pos, pos );
    layers.push_back(Layer());
    endInsertRows();
    activateLayer(pos);
}

void LayerList::deleteLayers(std::vector<int> indices){
    int count = 0;
    foreach(int i, indices){
        int idx = i - count;
        beginRemoveRows(QModelIndex(), idx, idx);
        layers.erase(layers.begin()+(idx));
        endRemoveRows();
        count++;
    }
}

/// Merge the layers listed
void LayerList::mergeLayers(std::vector<int> indices){
    if(indices.size() < 2)
        return;

    int dest_idx = indices[0];
    std::vector<int> & dest = layers[dest_idx].index;
    layers[dest_idx].copyFromGPU();
    indices.erase(indices.begin());

    // Copy every layer to the first layer
    foreach(int i, indices){
        layers[i].copyFromGPU();

        // Copy each individual index
        for(int j = 0; j < dest.size(); j++){
            // Dest must be -1 and source must not be -1
            if(dest[j] != -1 || layers[i].index[j] == -1){
                continue;
            }
            dest[j] = layers[i].index[j];
        }

    }

    deleteLayers(indices);
    layers[dest_idx].copyToGPU();
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

void LayerList::activateLayer(int i){
    emit selectLayer(i);
    layers[i].active = true;
}
