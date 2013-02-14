#ifndef LAYERLIST_H
#define LAYERLIST_H

#include <vector>
#include <memory>
#include <QAbstractListModel>
#include "model/layer.h"

class LayerList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit LayerList(QObject *parent = 0);
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    std::shared_ptr<Layer> addLayer();
    std::shared_ptr<Layer> addLayer(QString name, QColor color = QColor(255, 255, 255));
    int16_t genLabelId(std::shared_ptr<Layer> layer);

 signals:
    void layerUpdate(int id);
    void lookupTableUpdate();

 public:
    int16_t last_label_id_;
    std::vector<std::shared_ptr<Layer> > layers_; // a layer is a group of labels
    std::map<int16_t, std::weak_ptr<Layer> > layer_lookup_table_; // label associated with layer
};

#endif // LAYERLIST_H
