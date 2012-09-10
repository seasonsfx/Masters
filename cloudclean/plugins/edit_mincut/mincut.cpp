#include "mincut.h"
#include <QDebug>

#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

MinCut::MinCut(CloudModel * cm, int source_layer_idx, int dest_layer_idx)
{
    source_layer = &cm->layerList.layers[source_layer_idx].index;
    dest_layer = &cm->layerList.layers[dest_layer_idx].index;

    kdtree = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

    pcl::IndicesConstPtr indices(source_layer);

    kdtree->setInputCloud(cm->cloud, indices);

    /*
    // Blank
    for(int i = 0; i < dlayer.size(); i++){
        dlayer[i] = -1;
    }

    // Commit changes
    for(int i : clusters[0].indices){
        dlayer[i] = i;
        slayer[i] = -1;
    }

    cm->layerList.layers[source_layer].cpu_dirty = true;
    cm->layerList.layers[dest_layer].cpu_dirty = true;

    cm->layerList.layers[source_layer].sync();
    cm->layerList.layers[dest_layer].sync();
*/

}

void MinCut::createGraph(int k){



    graph.reset();
    graph = boost::shared_ptr<Graph>(new Graph());


    IndexMap vertex_index_map = get(boost::vertex_index, *graph);
    WeightMap edge_weight_map = get(boost::edge_weight, *graph);


    // Create vertices
    for(int i : *source_layer){
        Vertex v = add_vertex(*graph);
        vertex_index_map[v] = i;
    }

    // Create egde
    for(int i : *source_layer){
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
        kdtree->nearestKSearch (kdtree->getInputCloud()->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance);
        // add vertex and its neighbours to graph

        for(int dest_idx : pointIdxNKNSearch)
            add_edge(i, dest_idx, *graph);



    }
}

