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

typedef std::set<std::weak_ptr<Layer>,
    std::owner_less<std::weak_ptr<Layer> > > LayerSet;

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
    int16_t genLabelId(std::shared_ptr<Layer> layer);

 signals:
    void layerUpdate(std::shared_ptr<Layer> layer);
    void lookupTableUpdate();
    void changedSelection(std::vector<int> selection);

 public slots:
    void selectionChanged(const QItemSelection &sel, const QItemSelection &des);
    void deleteLayer(int idx);
    void deleteLayer();

 private:
    std::mutex * mtx_;

 public:
    int last_label_;
    std::vector<int> selection_;
    std::shared_ptr<Layer> default_layer_;
    std::vector<std::shared_ptr<Layer> > layers_; // a layer is a group of labels
    std::map<uint8_t, LayerSet> layer_lookup_table_; // label associated with layer
};

#endif // LAYERLIST_H
