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

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_LAYERVIEW_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_LAYERVIEW_H_

#include <vector>

#include <QDockWidget>
#include <QModelIndex>
#include <QAbstractItemView>

#include "cloudclean/cloudclean_global.h"

class QItemSelection;
class CloudModel;

namespace Ui {
    class LayerView;
}

class DLLSPEC LayerView : public QDockWidget {
    Q_OBJECT
    
 public:
    explicit LayerView(QWidget *parent, CloudModel * cm);
    ~LayerView();
    
 private:
    CloudModel * cm;
    Ui::LayerView *ui;
    std::vector<int> getSelection();

 public slots:
    void setSelectionMode(QAbstractItemView::SelectionMode mode);
    void selectionChanged(const QItemSelection & sel, const QItemSelection & des);
    void selectLayer(int i);
    void deleteLayers();
    void mergeLayers();

 signals:
    void updateView();
};

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_LAYERVIEW_H_