#ifndef LASSO_H
#define LASSO_H

#include <vector>
#include <Eigen/Dense>

#include "glarea.h"

class Lasso
{
public:
    Lasso();

    // add a new point in normalised screen coordinates
    void addPoint(Eigen::Vector2f point);
    void drawLasso(Eigen::Vector2f mouseLoc, GLArea * glarea);
    void clear();
    std::vector<Eigen::Vector2f> getPolygon();
    void getIndices(Eigen::Matrix4f & ndc_mat,
                    pcl::PointCloud<pcl::PointXYZI> *cloud,
                    std::vector<int> & source,
                    std::vector<int> & dest);

private:
    // Points normalised
    std::vector<Eigen::Vector2f>    points;
};

#endif // LASSO_H
