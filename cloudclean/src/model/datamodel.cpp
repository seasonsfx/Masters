#include "model/datamodel.h"

DataModel::DataModel() {
    last_cloud_id_ = -1;
    last_layer_id_ = -1;
    last_label_id_ = -1;
}

int DataModel::addCloud() {
    clouds_[++last_cloud_id_].reset(new PointCloud());
    emit cloudUpdate(last_cloud_id_);
    return last_cloud_id_;
}

int DataModel::addCloud(const char* filename) {
    clouds_[++last_cloud_id_].reset(new PointCloud());
    clouds_[last_cloud_id_]->load_ptx(filename);
    emit cloudUpdate(last_cloud_id_);
    return last_cloud_id_;
}

int DataModel::addCloud(std::shared_ptr<PointCloud> pc){
    clouds_[++last_cloud_id_] = pc;
    emit cloudUpdate(last_cloud_id_);
    return last_cloud_id_;
}

int DataModel::addLayer(){
    layers_[++last_layer_id_] = Layer();
    emit layerUpdate(last_layer_id_);
    return last_layer_id_;
}

int DataModel::addLayer(QString name, QColor color){
    layers_[++last_layer_id_] = Layer();
    layers_[last_layer_id_].name_ = name;
    layers_[last_layer_id_].color_ = color;
    emit layerUpdate(last_layer_id_);
    return last_layer_id_;
}

int16_t DataModel::genLabelId(int layer_id){
    layer_lookup_table_[++last_label_id_] = layer_id;
    //emit lookupTableUpdate();
    return last_label_id_;
}

