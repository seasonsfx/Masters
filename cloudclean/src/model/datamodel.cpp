#include "model/datamodel.h"

DataModel::DataModel() {
    last_cloud_id = -1;
}

int DataModel::addCloud() {
    clouds[++last_cloud_id] = PointCloud();
    return last_cloud_id;
}

int DataModel::addCloud(const char* filename) {
    clouds[addCloud()].load_ptx(filename);
    return last_cloud_id;
}
