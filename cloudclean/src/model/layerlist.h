#ifndef LAYERLIST_H
#define LAYERLIST_H

#include <QAbstractListModel>
#include "model/layer.h"


class LayerList : public QAbstractListModel
{
    Q_OBJECT
 public:
    explicit LayerList(QObject *parent = 0);
    
 signals:
    
 public slots:

 private:
    int last_layer_id_;
    int16_t last_label_id_;

    std::map<int, Layer> layers_; // a layer is a group of labels
    std::map<int, int> layer_lookup_table_; // layer associated with label
    
};

#endif // LAYERLIST_H
