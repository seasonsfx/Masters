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

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_LAYERLIST_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_LAYERLIST_H_

#include <QAbstractListModel>
#include <QAbstractItemView>
#include "layer.h"  // includes gl stuff
#include <vector>

#include "cloudclean/cloudclean_global.h"

class DLLSPEC LayerList : public QAbstractListModel {
    Q_OBJECT

 public:
    static const int CREATE_NEW_LAYER = 0;
    static const int ADD_TO_ACTIVE_LAYER = 1;
    static const int REMOVE_POINTS = 2;

    int newLayerMode;

    explicit LayerList(QObject *parent = 0);
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    int columnCount(const QModelIndex & parent) const;
    Qt::ItemFlags flags(const QModelIndex & index) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    bool setData(const QModelIndex & index, const QVariant & value, int role);
    void newLayer();

    void reset();
    std::vector<Layer> layers;
    void activateLayer(int i);
    void toggleVisible(int i);
    void setSelectMode(QAbstractItemView::SelectionMode mode);

 signals:
    void selectModeChanged(QAbstractItemView::SelectionMode mode);
    void selectLayer(int i);
    void updateView();

 public slots:
    void deleteLayers(std::vector<int> indices);
    void mergeLayers(std::vector<int> layersToMerge);
    void selectModeChanged(int index);
};

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_LAYERLIST_H_
