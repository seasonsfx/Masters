#ifndef LASSO_H
#define LASSO_H

#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "glheaders.h"
#include "utilities/export.h"

class QPaintDevice;

class UTIL_API Lasso
{
public:
    Lasso();

    // add a new point in normalised screen coordinates
    void addPoint(int x, int y, QPaintDevice *device);
    void movePoint(int x, int y, QPaintDevice *device);
    void addPoint(Eigen::Vector2f point);
    void drawLasso(int x, int y, QPaintDevice *device);
    void drawLasso(Eigen::Vector2f mouseLoc, QPaintDevice *device);
    void clear();
    std::vector<Eigen::Vector2f> getPolygon();
    void getIndices(Eigen::Matrix4f & ndc_mat,
                    pcl::PointCloud<pcl::PointXYZI> *cloud,
                    std::shared_ptr<std::vector<int> > source_indices,
                    std::shared_ptr<std::vector<int> > removed_indices);
    void getIndices2D(int height, const Eigen::Affine2f & cam,
                      const std::vector<int> &cloud_to_grid_map,
                      std::shared_ptr<std::vector<int> > source_indices,
                      std::shared_ptr<std::vector<int> > removed_indices);

private:
    // Points normalised
    std::vector<Eigen::Vector2f>    points;
};

#endif // LASSO_H
