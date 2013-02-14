#ifndef MODEL_CDATAMODEL_H_
#define MODEL_CDATAMODEL_H_

#include <map>
#include <vector>
#include "model/layer.h"
#include "model/pointcloud.h"

class DataModel: public QObject {
    Q_OBJECT
 public:
    DataModel();
    int addCloud();
    int addCloud(const char* filename);
    int addCloud(std::shared_ptr<PointCloud> pc);
    int addLayer();
    int addLayer(QString name, QColor color = QColor(255, 255, 255));
    int16_t genLabelId(int layer_id);

 signals:
    void layerUpdate(int id);
    void cloudUpdate(int id);
    void lookupTableUpdate();

 public:
    int last_cloud_id_;
    int last_layer_id_;
    int16_t last_label_id_;

    std::map<int, Layer> layers_; // a layer is a group of labels
    std::map<int, int> layer_lookup_table_; // layer associated with label

    std::map<int, std::shared_ptr<PointCloud> > clouds_;
};

#endif  // MODEL_CDATAMODEL_H_
