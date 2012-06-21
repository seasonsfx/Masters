#ifndef APPDATA_H
#define APPDATA_H

#include <QObject>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include "layerlist.h"

class AppData : public QObject
{
    Q_OBJECT
public:

    int K;
    float radius;

    std::vector< int >                              cloud_to_grid_map;
    int     x_dim;
    int     y_dim;

    pcl::PointCloud<pcl::PointXYZI>::Ptr            cloud;
    LayerList                                       layerList;

    //pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr           kdtree;
    //pcl::PointCloud<pcl::FPFHSignature33>::Ptr      fpfhs;
    //pcl::PointCloud<pcl::Normal>::Ptr               normals;

    // Fetch singleton
    static AppData* Instance();
    bool loadFile(const char * input_file, int subsample);
    bool saveFile(const char * output_file);

private:
    static AppData* only_instance;
    explicit AppData(QObject *parent = 0);

    // Disable copy and assignment
    AppData(AppData const&): QObject(0){}
    AppData& operator=(AppData const&) { return *this; }

signals:

public slots:

};

#endif // APPDATA_H
