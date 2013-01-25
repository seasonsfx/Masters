#ifndef CLOUDCLEAN_CPOINTCLOUD_H_
#define CLOUDCLEAN_CPOINTCLOUD_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/*
enum PointAttributes{
    visible = 0x001,
    selected = 0x002
};
*/

class CPointCloud : public pcl::PointCloud<pcl::PointXYZI> {
 public:
    explicit CPointCloud();

 public:
    std::vector<int> cloud_to_grid_map;
    int scan_width;
    int scan_height;

    std::vector<int16_t> labels;
    // std::vector<int8_t> attribute_flags;

};

#endif // CLOUDCLEAN_CPOINTCLOUD_H_
