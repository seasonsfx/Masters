#ifndef MINCUT_H
#define MINCUT_H

#include "cloudmodel.h"

#include <vector>

#include <Eigen/Dense>

#include <boost/make_shared.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/search/search.h>

class MinCut
{
public:
    MinCut(CloudModel *cm, int index, int source_layer, int dest_layer);

private:
    void createGraph();

public:

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    vector<int> * source_layer;
    int source;
};

#endif // MINCUT_H
