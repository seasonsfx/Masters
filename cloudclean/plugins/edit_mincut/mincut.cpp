#include "mincut.h"

#include <utility>
#include <map>
#include <math.h>

#include <QDebug>

//#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

MinCut::MinCut(CloudModel * cm, int source_layer_idx, int dest_layer_idx)
{
    qDebug("Source: %d, Dest %d", source_layer_idx, dest_layer_idx);
    this->cm = cm;
    source = &cm->layerList.layers[source_layer_idx];
    dest = &cm->layerList.layers[dest_layer_idx];

    source_layer = &cm->layerList.layers[source_layer_idx].index;
    dest_layer = &cm->layerList.layers[dest_layer_idx].index;

    kdtree = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

    kdtree->setInputCloud(cm->cloud, pcl::IndicesConstPtr(new std::vector<int>(*source_layer)));

}

void MinCut::createGraph(int k, pcl::PointXYZI & clickPoint){
    qDebug("Creating graph");
    graph.reset();
    graph = boost::shared_ptr<Graph>(new Graph());

    // Property maps, vertices and edges are keys to these things and return graph properties
    IndexMap vertex_index_map = get(boost::vertex_index2, *graph);
    WeightMap edge_weight_map = get(boost::edge_weight, *graph);

    // So we know which index values in the cloud, maps to which vertices in the graph
    std::map<int, Vertex> IVMap;

    qDebug("Adding vertices");
    // Create vertices
    for(int i : *source_layer){
        if(i == -1)
            continue;
        Vertex v = add_vertex(*graph);
        IVMap[i] = v;
        vertex_index_map[v] = i;
    }


    qDebug("Adding edges");
    // for each vertex add edges
    for (std::pair<vertex_iter, vertex_iter> vp = vertices(*graph); vp.first != vp.second; ++vp.first) {

        // Storage for neighbours
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        Vertex source_vertex = *vp.first;
        int source_vertex_idx = vertex_index_map[source_vertex];

        // Find neighbours
        kdtree->nearestKSearch (kdtree->getInputCloud()->points[source_vertex_idx], k, pointIdxNKNSearch, pointNKNSquaredDistance);

        // Add all the neighbours
        for(int i = 0; i < pointIdxNKNSearch.size(); i++){

            int dest_vertex_idx = pointIdxNKNSearch[i];

            Vertex dest_vertex = IVMap[dest_vertex_idx];
            Edge edge = (add_edge(source_vertex, dest_vertex, *graph)).first;

            // need to assign weight also I think
            //float weight = pointNKNSquaredDistance[i];

            pcl::PointXYZI & destPoint = cm->cloud->points[dest_vertex_idx];



            float dist = sqrt( pow(clickPoint.x - destPoint.x, 2) +
                               pow(clickPoint.y - destPoint.y, 2));

            edge_weight_map[edge] = 1/dist;
            //qDebug("Edge from %d to %d", source_vertex_idx, dest_vertex_idx);
        }

    }


}

float MinCut::segment(){
    qDebug("Start segmentation");
    IndexMap vertex_index_map = get(boost::vertex_index2, *graph);

    qDebug("Making map");
    auto parities = boost::make_one_bit_color_map(num_vertices(*graph), get(boost::vertex_index, *graph));
    qDebug("Stoer wagner");
    float w = boost::stoer_wagner_min_cut(*graph, get(boost::edge_weight, *graph), boost::parity_map(parities));

    qDebug("Saving");

    // Apply changes

    for (std::pair<vertex_iter, vertex_iter> vp = vertices(*graph); vp.first != vp.second; ++vp.first) {
            Vertex v = *vp.first;
            int idx = vertex_index_map[v];
            if (get(parities, v)){
                dest_layer->at(idx) = idx;
                source_layer->at(idx) = -1;
            }
            else{
                dest_layer->at(idx) = -1;
                source_layer->at(idx) = idx;
            }

    }

    source->copyToGPU();
    dest->copyToGPU();

}

