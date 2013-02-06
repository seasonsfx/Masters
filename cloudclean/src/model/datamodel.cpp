#include "model/datamodel.h"

DataModel::DataModel() {
    last_cloud_id_ = -1;
    last_layer_id_ = -1;
    last_label_id_ = -1;
    layer_lookup_table_dirty_ = true;
}

int DataModel::addCloud() {
    clouds_[++last_cloud_id_] = PointCloud();
    return last_cloud_id_;
}

int DataModel::addCloud(const char* filename) {
    clouds_[addCloud()].load_ptx(filename);
    return last_cloud_id_;
}

int DataModel::addLayer(){
    layers_[++last_layer_id_] = Layer();
    return last_layer_id_;
}

int DataModel::addLayer(QString name, QColor color){
    layers_[addLayer()].name_ = name;
    layers_[last_layer_id_].color_ = color;
    return last_layer_id_;
}

int16_t DataModel::genLabelId(){
    return ++last_label_id_;
}

