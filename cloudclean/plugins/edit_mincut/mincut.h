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

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/typeof/typeof.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
   boost::property< boost::vertex_index_t, int>, boost::property<boost::edge_weight_t, int> > Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;
typedef boost::property_traits<WeightMap>::value_type EdgeWeight;

typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
typedef boost::property_traits<IndexMap>::value_type Index;


class MinCut
{
public:
    MinCut(CloudModel *cm, int source_layer_idx, int dest_layer_idx);

private:
    void createGraph(int k);
    void setWeights(int source_idx, int sink_idx);
    float segment();


public:

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree;
    vector<int> * source_layer;
    vector<int> * dest_layer;
    boost::shared_ptr<Graph> graph;
};

#endif // MINCUT_H
