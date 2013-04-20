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
#include "model/export.h"

class MODEL_DLLSPEC LayerList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit LayerList(QObject *parent = 0);
    ~LayerList();
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    std::shared_ptr<Layer> addLayer(std::shared_ptr<Layer> layer);
    std::shared_ptr<Layer> addLayerWithId(uint id);
    std::shared_ptr<Layer> addLayer();
    std::shared_ptr<Layer> addLayer(QString name);
    int16_t newLabelId();

    const LayerSet &getLayersForLabel(int i);
    void copyLayerSet(uint8_t source_label, uint8_t dest_label);

    int getLayerIndex(std::shared_ptr<Layer> layer);

 signals:
    void layerUpdate(std::shared_ptr<Layer> layer);
    void lookupTableUpdate();
    void changedSelection(std::vector<std::weak_ptr<Layer> > selection);

 public slots:
    void selectionChanged(const QItemSelection &sel, const QItemSelection &des);
    void deleteLayer(std::shared_ptr<Layer> layer);
    void deleteLayer();
    void deleteLayer(int idx);

 private:
    std::mutex * mtx_;
    std::map<uint16_t, LayerSet> layer_lookup_table_; // label associated with layer
    std::vector<uint16_t> free_labels_;

 public:
    uint last_label_;
    std::vector<std::weak_ptr<Layer> > selection_;
    std::shared_ptr<Layer> default_layer_;
    std::vector<std::shared_ptr<Layer> > layers_; // a layer is a group of labels
    std::map<uint, std::shared_ptr<Layer> > layer_id_map_;
};

#endif // LAYERLIST_H
