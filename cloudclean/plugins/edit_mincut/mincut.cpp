#include "mincut.h"
#include <QDebug>

//#include <pcl/segmentation/min_cut_segmentation.h>
//#include <pcl/visualization/cloud_viewer.h>

MinCut::MinCut(CloudModel * cm, int index, int source_layer, int dest_layer)
{
    std::vector<int> & slayer = cm->layerList.layers[source_layer].index;
    std::vector<int> & dlayer = cm->layerList.layers[dest_layer].index;

    pcl::IndicesPtr indices (&slayer);

    pcl::MinCutSegmentation<pcl::PointXYZI> seg;
    seg.setInputCloud (cm->cloud);
    seg.setIndices (indices);


    pcl::PointCloud<pcl::PointXYZI>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointXYZI point = cm->cloud->at(index);
    foreground_points->points.push_back(point);
    seg.setForegroundPoints (foreground_points);

    seg.setSigma (0.25);
    seg.setRadius (3.0433856);
    seg.setNumberOfNeighbours (14);
    seg.setSourceWeight (0.8);

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);

    qDebug() << "Maximum flow is " << seg.getMaxFlow ();

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

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ()){}

}

void MinCut::createGraph(){

}

