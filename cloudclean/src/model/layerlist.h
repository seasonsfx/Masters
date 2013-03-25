#ifndef LAYERLIST_H
#define LAYERLIST_H

#include <vector>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>
#include <QAbstractListModel>
#include <QItemSelection>
#include "model/layer.h"

class LayerList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit LayerList(QObject *parent = 0);
    ~LayerList();
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    std::shared_ptr<Layer> addLayer(std::shared_ptr<Layer> layer);
    std::shared_ptr<Layer> addLayer();
    std::shared_ptr<Layer> addLayer(QString name);
    int16_t newLabelId(std::shared_ptr<Layer> layer);

    const LayerSet &getLayersForLabel(int i);
    void copyLayerSet(uint8_t source_label, uint8_t dest_label);

 signals:
    void layerUpdate(std::shared_ptr<Layer> layer);
    void lookupTableUpdate();
    void changedSelection(std::vector<int> selection);

 public slots:
    void selectionChanged(const QItemSelection &sel, const QItemSelection &des);
    void mergeSelectedLayers();
    void intersectSelectedLayers();
    void deleteLayer(int idx);
    void deleteLayer();

 private:
    std::mutex * mtx_;
    std::map<uint8_t, LayerSet> layer_lookup_table_; // label associated with layer

 public:
    uint last_label_;
    std::vector<int> selection_;
    std::shared_ptr<Layer> default_layer_;
    std::vector<std::shared_ptr<Layer> > layers_; // a layer is a group of labels
};

#endif // LAYERLIST_H
