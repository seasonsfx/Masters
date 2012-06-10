#ifndef APPDATA_H
#define APPDATA_H

#include <QObject>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>

class AppData : public QObject
{
    Q_OBJECT
public:

    int vals_in_range;
    int K;
    float radius;
    int invalid_points;

    pcl::PointCloud<pcl::PointXYZI>::Ptr            cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr           kdtree;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr      fpfhs;
    pcl::PointCloud<pcl::Normal>::Ptr               normals;

    boost::shared_ptr<std::vector<int> >            p_valid_indices;

    std::vector<pcl::FPFHSignature33> stats;
    pcl::FPFHSignature33 mean;
    pcl::FPFHSignature33 stdev;

    // Fetch singleton
    static AppData* Instance();
    bool loadFile(const char * input_file, int subsample);

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
