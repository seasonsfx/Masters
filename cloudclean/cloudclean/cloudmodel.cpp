#include "cloudmodel.h"
#include <ctime>
#include "io.h"
#include <pcl/filters/filter.h>
#include <QDebug>
#include <QTime>
#include <GL/glu.h>

void inline  glError(const char * msg){
    int err = glGetError();
    if(err){
        printf("%s : %s\n", msg , gluErrorString(err));
    }
}

CloudModel* CloudModel::only_instance = NULL;

CloudModel::CloudModel(QObject *parent) :
    QObject(parent), point_buffer( QGLBuffer::VertexBuffer), normal_buffer(QGLBuffer::VertexBuffer)
{
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    x_dim = 0;
    y_dim = 0;
    loaded = false;
}

CloudModel * CloudModel::Instance(){
    if(!only_instance)
        only_instance = new CloudModel();
    return only_instance;
}

bool CloudModel::saveFile(const char * output_file){

    // Reconstruct original cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    original_cloud->is_dense = false;
    original_cloud->width = x_dim;
    original_cloud->height = y_dim;
    original_cloud->resize(x_dim*y_dim);
    original_cloud->sensor_origin_[0] = cloud->sensor_origin_[0];
    original_cloud->sensor_origin_[1] = cloud->sensor_origin_[1];
    original_cloud->sensor_origin_[2] = cloud->sensor_origin_[2];
    original_cloud->sensor_origin_[3] = cloud->sensor_origin_[3];
    original_cloud->sensor_orientation_ = cloud->sensor_orientation_;

    pcl::PointXYZI p;
    p.x = p.y = p.z = p.intensity = NAN;

    /// Init with nans
    for(unsigned int i = 0; i < cloud_to_grid_map.size(); i++){
        original_cloud->points[i] = p;
    }


    // Copy in all visible layers
    std::vector<Layer> & layers = layerList.layers;
    for(unsigned int l = 0; l < layers.size(); l++){
        printf("l: %d\n", l);
        if(!layers[l].visible)
            continue;
        layers[l].copyFromGPU();
        for(unsigned int i = 0; i < layers[l].index.size(); i++){
            printf("i: %d\n", i);
            int idx = layers[l].index[i];
            // for some reason bad indices sneak in here
            // that are not -1
            if(idx == -1)
                continue;
            original_cloud->points[cloud_to_grid_map[idx]] = cloud->points[idx];
        }
    }


    save_ptx(output_file, original_cloud);
    return true;
}

bool CloudModel::createBuffers(){

    if(!loaded)
        return false;

    point_buffer.create();
    point_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    if ( !point_buffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return false;
    }
    point_buffer.allocate(cloud->points.size() * sizeof(float) * 4);

    float data[4];
    for (int i = 0; i < (int)cloud->size(); i++)
    {
        data[0] = cloud->points[i].x;
        data[1] = cloud->points[i].y;
        data[2] = cloud->points[i].z;
        data[3] = cloud->points[i].intensity;
        int offset = 4*sizeof(float)*i;
        point_buffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
    }

    point_buffer.release();

    /// Create first layer
    layerList.newLayer();

    Layer & layer = layerList.layers[0];

    // Change initial layers colour
    layer.colour = Eigen::Vector3f(1.0f, 1.0f, 1.0f);

    /*layer.colour = Eigen::Vector3f(1.0f, 0.5f, 0.5f);
    layer.gl_index_buffer.create();
    layer.gl_index_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    layer.gl_index_buffer.bind();
    layer.gl_index_buffer.allocate(cloud->size() * sizeof(int) );
*/

    /// Initialise the first layer to include all points
    for(unsigned int i = 0; i < cloud->size(); i++){
        layer.index[i] = i;
        //layer.gl_index_buffer.write(i*sizeof(int), reinterpret_cast<const void *>(&i), sizeof(int));
    }

    layer.cpu_dirty = true;
    layer.sync();

    //layer.gl_index_buffer.release();

    // Comment this out if all works
    // Load normals to gpu
/*
    normal_buffer.create();
    normal_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    assert(normal_buffer.bind());
    normal_buffer.allocate(cloud->points.size() * sizeof(float) * 6);

    for (int i = 0; i < (int)cloud->size(); i++)
    {
        float data2[6];
        data2[0] = cloud->points[i].x;
        data2[1] = cloud->points[i].y;
        data2[2] = cloud->points[i].z;
        data2[3] = data2[0]+(normals->at(i).data_n[0]*0.05);
        data2[4] = data2[1]+(normals->at(i).data_n[1]*0.05);
        data2[5] = data2[2]+(normals->at(i).data_n[2]*0.05);

        int offset = 6*sizeof(float)*i;
        normal_buffer.write(offset, reinterpret_cast<const void *> (data2), sizeof(data2));

        float p[6] = {-1,-1,-1,-1,-1,-1};
        normal_buffer.read(offset, reinterpret_cast<void *>(p), sizeof(p));
    }

    normal_buffer.release();
*/
    qDebug("Buffers created & loaded.");
    return true;
}

void normal_estimation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

bool CloudModel::loadFile(const char * input_file, int subsample){

    qDebug("Subsample: %d\n", subsample);

    QTime t;
    t.start();

    cloud = read_ptx(input_file, subsample);

    qDebug("File loaded in %d ms", t.elapsed());

    x_dim = cloud->width;
    y_dim = cloud->height;

    t.start();
    /// Calculate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_tmp(new pcl::PointCloud<pcl::Normal> ());

    normal_estimation(cloud, normals_tmp);
    qDebug("Normals calculated in %d ms", t.elapsed());

    t.start();
    /// Filter and flatten point cloud
    pcl::removeNaNFromPointCloud(*cloud, *cloud, cloud_to_grid_map);
    qDebug("Cloud filtered in %d ms", t.elapsed());

    t.start();
    /// Move normals to unstructured cloud
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal> ());
    normals->resize(cloud->size());
    for(int i = 0; i < cloud_to_grid_map.size(); i++){
        normals->points[i] = normals_tmp->points[cloud_to_grid_map[i]];
    }

    qDebug("Normals moved in  %d ms", t.elapsed());

    /*
    for(int i = 0; i < normals_tmp->size(); i++){
        printf("normal: (%f, %f, %f)\n",
                normals_tmp->points.at(i).data_n[0],
                normals_tmp->points.at(i).data_n[1],
                normals_tmp->points.at(i).data_n[2]);
    }
    */

    /*for(int i = 0; i < normals->size(); i++){
        printf("normal: (%f, %f, %f)\n",
                normals->points.at(i).data_n[0],
                normals->points.at(i).data_n[1],
                normals->points.at(i).data_n[2]);
    }
    */

    /*
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.05, 0.05, 0.05);
    viewer.addPointCloudNormals<pcl::PointXYZI,pcl::Normal>(cloud, normals);

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1);

    while (!viewer.wasStopped ()){
        viewer.spinOnce ();
    }
    */

    if(loaded)
        layerList.reset();

    loaded = true;

    t.start();
    createBuffers();
    qDebug("Points loaded to GPU in %d ms", t.elapsed());

    return true;
}

bool CloudModel::isLoaded(){
    return loaded;
}

class PointIdx{
public:
    int x;
    int y;
    PointIdx(int x, int y): x(x), y(y){}
    PointIdx(): x(0), y(0){}
};

class PointIdxPair{
public:
    PointIdx p1;
    PointIdx p2;
    PointIdxPair(PointIdx p1, PointIdx p2): p1(p1), p2(p2){}
    PointIdxPair(int p1x, int p1y, int p2x, int p2y): p1(PointIdx(p1x, p1y)), p2(PointIdx(p2x, p2y)){}
};

void normal_estimation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
    int length = cloud->points.size();
    int width = cloud->width;
    int height = cloud->height;
    normals->points.resize(length);

    for (int i = 0; i < length; ++i){

        // Don't calculate normal for NANs
        if (cloud->points[i].x != cloud->points[i].x)
        {
            normals->points[i].data_n[0] = NAN;
            normals->points[i].data_n[1] = NAN;
            normals->points[i].data_n[2] = NAN;
            normals->points[i].data_n[3] = NAN;
            continue;
        }

        // Define all pairs of indices relative to current position


        // Array of ponits that form a traingle along with the center point
        PointIdxPair trianglePairs[8] = {
            PointIdxPair(PointIdx(0, -1), PointIdx(-1, -1)),
            PointIdxPair(PointIdx(1, -1), PointIdx(0, -1)),
            PointIdxPair(PointIdx(-1, -1), PointIdx(-1, 0)),
            PointIdxPair(PointIdx(1, 0), PointIdx(1, -1)),
            PointIdxPair(PointIdx(-1, 0), PointIdx(-1, 1)),
            PointIdxPair(PointIdx(1, 1), PointIdx(1, 0)),
            PointIdxPair(PointIdx(-1, 1), PointIdx(0, 1)),
            PointIdxPair(PointIdx(0, 1), PointIdx(1, 1))
        };

        // Normal calculation vars
        Eigen::Vector3f agregate_n(0.0f,0.0f,0.0f);
        int face_count = 0;

        if(i == 82455){
            int k = 0;
        }

        // Calculate normal from 8 triangles
        for (int j = 0; j < 8; ++j)
        {
            PointIdxPair & relativePair = trianglePairs[j];

            // calculate current x and y
            int cur_x = i%width;
            int cur_y = i/width;

            bool outbounds = false;

            // is p1 out of bounds
            int x_pos = cur_x + relativePair.p1.x;
            int y_pos = cur_y + relativePair.p1.y;
            if(x_pos > width-1 || x_pos < 0)
                outbounds = true;
            if(y_pos > height-1 || y_pos < 0)
                outbounds = true;

            // is p2 out of bounds
            x_pos = cur_x + relativePair.p2.x;
            y_pos = cur_y + relativePair.p2.y;
            if(x_pos > width-1 || x_pos < 0)
                outbounds = true;
            if(y_pos > height-1 || y_pos < 0)
                outbounds = true;

            if (!outbounds){

                // Calculate absolute indices
                int idx1 = (cur_y+relativePair.p1.y)*width + (cur_x+relativePair.p1.x);
                int idx2 = (cur_y+relativePair.p2.y)*width + (cur_x+relativePair.p2.x);

                // Ignore NAN neighbours
                if(cloud->points[idx1].x != cloud->points[idx1].x)
                    continue;
                if(cloud->points[idx2].x != cloud->points[idx2].x)
                    continue;

                pcl::PointXYZI & cp0 = cloud->points[i];
                pcl::PointXYZI & cp1 = cloud->points[idx1];
                pcl::PointXYZI & cp2 = cloud->points[idx2];

                Eigen::Vector3f p0(cp0.x, cp0.y, cp0.z);
                Eigen::Vector3f p1(cp1.x, cp1.y ,cp1.z);
                Eigen::Vector3f p2(cp2.x, cp2.y ,cp2.z);

                // Cross product sometimes 0 0 0
                Eigen::Vector3f tmp = ((p2 - p0).cross(p1 - p0));
                tmp.normalize();

                // Normalized 0 0 0 == NaN
                if(tmp.x() != tmp.x() ||
                   tmp.y() != tmp.y() ||
                   tmp.z() != tmp.z()
                )
                    continue;

                agregate_n= agregate_n + tmp;
                face_count++;

             }

        }

        // Face count can be 0 or normals can cancel
        if(face_count == 0){
            normals->points[i].data_n[0] = NAN;
            normals->points[i].data_n[1] = NAN;
            normals->points[i].data_n[2] = NAN;
            normals->points[i].data_n[3] = NAN;
            continue;
        }

        Eigen::Vector3f normal = agregate_n.normalized();

        pcl::flipNormalTowardsViewpoint (cloud->points[i], 0,0,0,normal);

        normals->points[i].data_n[0] = normal(0);
        normals->points[i].data_n[1] = normal(1);
        normals->points[i].data_n[2] = normal(2);
        normals->points[i].data_n[3] = 0.0f;

        assert(normals->points[i].data_n[0] == normals->points[i].data_n[0]);

    }
}
