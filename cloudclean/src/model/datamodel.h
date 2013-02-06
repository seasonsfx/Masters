#ifndef MODEL_CDATAMODEL_H_
#define MODEL_CDATAMODEL_H_

#include <map>
#include <vector>
#include "model/layer.h"
#include "model/selection.h"
#include "model/pointcloud.h"

class DataModel {
 public:
    DataModel();
    int addCloud();
    int addCloud(const char* filename);
    int addLayer();
    int addLayer(QString name, QColor color = QColor(255, 255, 255));
    int16_t genLabelId();
    // save everything as a flat cloud?

 public:
    int last_cloud_id_;
    int last_layer_id_;
    int16_t last_label_id_;
    std::map<int, Layer> layers_;
    std::map<int, int> layer_lookup_table_; // layer associated with color
    Selection selection_;
    std::map<int, PointCloud> clouds_;
    bool layer_lookup_table_dirty_;
};

#endif  // MODEL_CDATAMODEL_H_
