#ifndef POINTPICKER_H
#define POINTPICKER_H

#include <Eigen/Dense>
#include <pcl/octree/octree.h>
#include "cloudclean_global.h"

class GLArea;
class CloudModel;

class DLLSPEC PointPicker
{
public:
    PointPicker(GLArea * glarea = nullptr, CloudModel *cm = nullptr,
                float max_dist = 0);
    int pick(int x, int y, int source_layer);

private:
    void getClickRay(int x, int y, Eigen::Vector3f& p1, Eigen::Vector3f& p2);

private:
    GLArea * glarea;
    CloudModel *cm;
    float max_dist;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr  octree;
};

#endif // POINTPICKER_H
