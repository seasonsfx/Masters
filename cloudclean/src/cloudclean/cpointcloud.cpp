#include "cpointcloud.h"

CPointCloud::CPointCloud()
    : pcl::PointCloud<pcl::PointXYZI>() {
    this->points;
}
