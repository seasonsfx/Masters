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

void LayerList::mergeLayers(std::vector<int> indices){
    std::vector<int> & dest = layers[indices[0]].index;
    for(int i = 1; i < indices.size(); i++){
        int idx = indices[i];
        layers[idx].copyFromGPU();
        for(int j = 0; j < dest.size(); j++){
            if(dest[j] != -1)
                continue;
            dest[j] = layers[idx].index[j];
        }
    }
    layers[indices[0]].copyToGPU();
    indices.erase(indices.begin());
    deleteLayers(indices);
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