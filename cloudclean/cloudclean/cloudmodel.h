#ifndef APPDATA_H
#define APPDATA_H

#include <QObject>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include "layerlist.h"

// TODO: a cloud model should probably own a GL context?
class CloudModel : public QObject
{
    Q_OBJECT
public:
    std::vector< int >                              cloud_to_grid_map;
    int                                             x_dim;
    int                                             y_dim;
    bool                                            loaded;

    // Where is the ground plane? Could be useful to detect trees

    pcl::PointCloud<pcl::PointXYZI>::Ptr            cloud;
    QGLBuffer                                       point_buffer;
    LayerList                                       layerList;
    pcl::PointCloud<pcl::Normal>::Ptr               normals;

public:
    static CloudModel* Instance();
    bool createBuffers();
    bool loadFile(const char * input_file, int subsample);
    bool saveFile(const char * output_file);
    bool isLoaded();

private:
    static CloudModel* only_instance;
    explicit CloudModel(QObject *parent = 0);
    CloudModel(CloudModel const&);
    CloudModel& operator=(CloudModel const&) { return *this; }

signals:

public slots:

};

#endif // APPDATA_H
