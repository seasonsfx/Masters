/*
 * Software License Agreement (BSD License)
 *
 *  CloudClean
 *  Copyright (c) 2013, Rickert Mulder
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rickert Mulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "layerlist.h"
#include <QTextStream>
#include "cloudmodel.h"
#include <QColor>

LayerList::LayerList(QObject *parent) :
    QAbstractListModel(parent)
{
    newLayerMode = 0;
}

int LayerList::rowCount ( const QModelIndex &) const {
    return layers.size();
}

int LayerList::columnCount ( const QModelIndex &) const {
    return 2;
}

Qt::ItemFlags LayerList::flags ( const QModelIndex & index ) const{

    int col = index.column();

    if(col == 0){
        return Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
    }

    return Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled;
}

QVariant LayerList::data ( const QModelIndex & index, int role ) const {
    int row = index.row();
    int col = index.column();

    if(col == 1){
        switch(role){
        case Qt::DisplayRole:
        {
            QString re;
            QTextStream(&re) << "Layer " << row;
            return re;
        }
        case Qt::DecorationRole:
            return QColor(layers[row].colour[0]*255, layers[row].colour[1]*255, layers[row].colour[2]*255);
        }
    }
    else{
        switch(role){
        /*case Qt::DisplayRole:
        {
            QString re;
            QTextStream(&re) << "Layer " << row;
            return re;
        }
        case Qt::DecorationRole:
            return QColor(layers[row].colour[0]*255, layers[row].colour[1]*255, layers[row].colour[2]*255);
        */
        case Qt::CheckStateRole:
                return layers[row].visible? Qt::Checked : Qt::Unchecked;
        }
    }
    return QVariant();

}


bool LayerList::setData(const QModelIndex & index, const QVariant & value, int role)
{
    int row = index.row();
    int col = index.column();

    if (role == Qt::CheckStateRole && col == 0)
    {
        toggleVisible(row);
    }
    return true;
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
void LayerList::mergeLayers(std::vector<int> layersToMerge){

    if(layersToMerge.size() < 2)
        return;

    int dest_idx = layersToMerge[0];
    std::vector<int> & dest = layers[dest_idx].index;

    layers[dest_idx].sync();

    // Delete first in list
    layersToMerge.erase(layersToMerge.begin());

    // Copy every layer to the first layer
    for(int i : layersToMerge){
        qDebug("Merging layer %d into %d", i, dest_idx);
        // Copy each individual index
        for(unsigned int j = 0; j < dest.size(); j++){
            // Dest must be -1 and source must not be -1
            if(dest[j] != -1 || layers[i].index[j] == -1){
                continue;
            }
            dest[j] = layers[i].index[j];
        }

    }

    deleteLayers(layersToMerge);
    layers[dest_idx].cpu_dirty = 1;
    layers[dest_idx].sync();
}

void LayerList::selectModeChanged(int index){
    newLayerMode = index;
    qDebug("Index: %d", index);
}

/// Watch out, has OpenGL context been initialized?
void LayerList::reset (){
    if(layers.size() < 2)
        return;
    layers.erase(layers.begin()+1, layers.end());
    Layer & layer = layers[0];

    CloudModel * app_data = CloudModel::Instance();

    layer.gl_index_buffer.bind();
    layer.gl_index_buffer.allocate(app_data->cloud->size()*sizeof(int));

    for(unsigned int i = 1; i < app_data->cloud->size(); i++){
        layer.index[i] = i;
    }
    layer.copyToGPU();
    layer.gl_index_buffer.release();
}

void LayerList::activateLayer(int i){
    emit selectLayer(i);
    layers[i].active = true;
}

void LayerList::toggleVisible(int i){
    layers[i].toggleVisible();
    QModelIndex mi = index(i, 0, QModelIndex());
    emit dataChanged(mi, mi);
    emit updateView();
}

void LayerList::setSelectMode(QAbstractItemView::SelectionMode mode){
    emit selectModeChanged(mode);
}
