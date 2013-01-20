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

#include "layerview.h"
#include <limits>
#include <vector>
#include <QItemSelectionModel>
#include "ui_layerview.h"
#include "cloudmodel.h"


LayerView::LayerView(QWidget *parent)
    : QDockWidget(parent), ui(new Ui::LayerView) {
    cm = CloudModel::Instance();
    ui->setupUi(this);
    ui->listView->horizontalHeader()->hide();
    ui->listView->verticalHeader()->hide();
    ui->listView->horizontalHeader()->setStretchLastSection(true);
    ui->listView->setGridStyle(Qt::NoPen);
    ui->listView->setModel(&CloudModel::Instance()->layerList);
    ui->listView->setSelectionMode(QAbstractItemView::MultiSelection);
    ui->listView->setColumnWidth(0, 30);
    ui->listView->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->selection_mode_combo->insertItem(0, "Create new layer");
    ui->selection_mode_combo->insertItem(1, "Add to active layer");
    ui->selection_mode_combo->insertItem(2, "Remove points");

    connect(ui->listView->selectionModel(), SIGNAL(
                selectionChanged(const QItemSelection &,
                                 const QItemSelection &)), this,
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));
    connect(ui->delete_button, SIGNAL(pressed()), this, SLOT(deleteLayers()));
    connect(ui->merge_button, SIGNAL(pressed()), this, SLOT(mergeLayers()));
    connect(ui->selection_mode_combo, SIGNAL(currentIndexChanged(int)),
            &cm->layerList, SLOT(selectModeChanged(int)));

    connect(&cm->layerList,
            SIGNAL(selectModeChanged(QAbstractItemView::SelectionMode)),
            this, SLOT(setSelectionMode(QAbstractItemView::SelectionMode)));
}

LayerView::~LayerView() {
    delete ui;
}

void LayerView::setSelectionMode(QAbstractItemView::SelectionMode mode) {
    ui->listView->setSelectionMode(mode);
}

void LayerView::selectionChanged(const QItemSelection & sel,
                                 const QItemSelection &des) {
    for (QModelIndex s : sel.indexes()) {
        int row = s.row();
        cm->layerList.layers[row].active = true;
    }
    for (QModelIndex s : des.indexes()) {
        int row = s.row();
        cm->layerList.layers[row].active = false;
    }
    emit updateView();
}

void LayerView::selectLayer(int i) {
    cm->layerList.layers[i].active = true;
    QItemSelectionModel * sm = ui->listView->selectionModel();
    QModelIndex mi = sm->model()->index(i, 0, QModelIndex());
    sm->select(mi, QItemSelectionModel::Select);
    mi = sm->model()->index(i, 1, QModelIndex());
    sm->select(mi, QItemSelectionModel::Select);
}

std::vector<int> LayerView::getSelection() {
    std::vector<int> re;
    QItemSelectionModel * sm = ui->listView->selectionModel();
    QModelIndexList sl = sm->selectedRows();
    for (QModelIndex mi : sl) {
        re.push_back(mi.row());
    }
    return re;
}

void LayerView::deleteLayers() {
    cm->layerList.deleteLayers(getSelection());
    emit updateView();
}

void LayerView::mergeLayers() {
    cm->layerList.mergeLayers(getSelection());
    emit updateView();
}
