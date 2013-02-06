#ifndef MODEL_CPOINTCLOUD_H_
#define MODEL_CPOINTCLOUD_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/*
enum PointAttributes{
    visible = 0x001,
    selected = 0x002
};
*/

class PointCloud : public pcl::PointCloud<pcl::PointXYZI> {
 public:
    explicit PointCloud();
    bool save_ptx(const char* filename);
    bool load_ptx(const char* filename, int subsample = 1);

 public:
    std::vector<int> cloud_to_grid_map;
    int scan_width;
    int scan_height;

    std::vector<int16_t> labels;
    // std::vector<int8_t> attribute_flags;


    // What needs to be in here?
    // Should normals be a default requirement
    // Maybe store attributes in a map and generate on the fly
};

#endif // MODEL_CPOINTCLOUD_H_
