#include "model/datamodel.h"

DataModel::DataModel() {
    last_cloud_id_ = -1;
    last_layer_id_ = -1;
    last_label_id_ = -1;
    layer_lookup_table_dirty_ = false;
    layers_dirty_ = false;
    clouds_dirty_ = false;
}

int DataModel::addCloud() {
    clouds_[++last_cloud_id_].reset(new PointCloud());
    clouds_dirty_ = true;
    return last_cloud_id_;
}

int DataModel::addCloud(const char* filename) {
    clouds_[++last_cloud_id_].reset(new PointCloud());
    clouds_[last_cloud_id_]->load_ptx(filename);
    clouds_dirty_ = true;
    return last_cloud_id_;
}

int DataModel::addCloud(std::shared_ptr<PointCloud> pc){
    clouds_[++last_cloud_id_] = pc;
    clouds_dirty_ = true;
    return last_cloud_id_;
}

int DataModel::addLayer(){
    layers_[++last_layer_id_] = Layer();
    layers_dirty_ = true;
    return last_layer_id_;
}

int DataModel::addLayer(QString name, QColor color){
    layers_[++last_layer_id_] = Layer();
    layers_[last_layer_id_].name_ = name;
    layers_[last_layer_id_].color_ = color;
    layers_dirty_ = true;
    return last_layer_id_;
}

int16_t DataModel::genLabelId(int layer_id){
    layer_lookup_table_[++last_label_id_] = layer_id;
    layer_lookup_table_dirty_ = true;
    return last_label_id_;
}

