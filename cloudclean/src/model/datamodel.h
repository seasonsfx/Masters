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
    // save everything as a flat cloud?

 public:
    int last_cloud_id;
    Layer layers;
    Selection selection;
    std::map<int, PointCloud> clouds;
};

#endif  // MODEL_CDATAMODEL_H_
