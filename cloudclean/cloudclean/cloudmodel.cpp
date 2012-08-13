#include "cloudmodel.h"
#include <ctime>
#include "io.h"
#include <pcl/filters/filter.h>
#include <QDebug>
#include <QTime>
#include <GL/glu.h>
#include <pcl/features/integral_image_normal.h>

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
        layers[l].sync();
        for(unsigned int i = 0; i < layers[l].index.size(); i++){
            //printf("i: %d\n", i);
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


    normal_buffer.create();
    normal_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    assert(normal_buffer.bind());
    normal_buffer.allocate(cloud->points.size() * sizeof(float) * 3);

    for (int i = 0; i < (int)cloud->size(); i++)
    {
        float data2[3];
        data2[0] = (normals->at(i).data_n[0]*0.1);
        data2[1] = (normals->at(i).data_n[1]*0.1);
        data2[2] = (normals->at(i).data_n[2]*0.1);

        int offset = 3*sizeof(float)*i;
        normal_buffer.write(offset, reinterpret_cast<const void *> (data2), sizeof(data2));
    }

    normal_buffer.release();

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
        data2[3] = data2[0]+(normals->at(i).data_n[0]*0.1);
        data2[4] = data2[1]+(normals->at(i).data_n[1]*0.1);
        data2[5] = data2[2]+(normals->at(i).data_n[2]*0.1);

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
void normal_estimation2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

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

    // Debug
    /*for(int i = 0; i < cloud->size(); i++){
        qDebug("Point (%f, %f, %f), Normal (%f, %f, %f)",
               cloud->points[i].x, cloud->points[i].y, cloud->points[i].z,
               normals_tmp->points[i].normal_x,
               normals_tmp->points[i].normal_y,
               normals_tmp->points[i].normal_z);
    }
    */

    t.start();
    /// Filter and flatten point cloud
    pcl::removeNaNFromPointCloud(*cloud, *cloud, cloud_to_grid_map);
    qDebug("Cloud filtered in %d ms", t.elapsed());


    t.start();
    /// Move normals to unstructured cloud
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal> ());
    normals->resize(cloud->size());
    int nans = 0;
    for(int i = 0; i < cloud_to_grid_map.size(); i++){
        normals->points[i] = normals_tmp->points[cloud_to_grid_map[i]];
        if(normals->points[i].normal_x != normals->points[i].normal_x)
            nans++;
    }

    qDebug("Missing normals: %d", nans);

    qDebug("Normals moved in  %d ms", t.elapsed());

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

void normal_estimation2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){

    // Need to save registration details and restore the,

    // save
    Eigen::Quaternionf sensor_orientation_old = cloud->sensor_orientation_;
    Eigen::Vector4f sensor_origin_old = cloud->sensor_origin_;

    // set
    cloud->sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0); // no rotation
    cloud->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);


    pcl::IntegralImageNormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // restore
    cloud->sensor_orientation_ = sensor_orientation_old;
    cloud->sensor_origin_ = sensor_origin_old;
}

bool isValidPoint(PointIdx & point, int width, int height, int index, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    // calculate current x and y
    int cur_x = index%width;
    int cur_y = index/width;

    // Bounds check
    int x_pos = cur_x + point.x;
    int y_pos = cur_y + point.y;
    if(x_pos > width-1 || x_pos < 0)
        return false;
    if(y_pos > height-1 || y_pos < 0)
        return false;

    // Index with NaN value is not valid
    int idx = y_pos*width + x_pos;
    if(cloud->points[idx].x != cloud->points[idx].x)
        return false;

    return true;
}

void normal_estimation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){

    int length = cloud->points.size();
    int width = cloud->width;
    int height = cloud->height;
    normals->points.resize(length);

    // Relative point anticlockwise around (0,0)
    PointIdx pointList[8] = {
        PointIdx(1, 0),
        PointIdx(1, -1),
        PointIdx(0, -1),
        PointIdx(-1, -1),
        PointIdx(-1, 0),
        PointIdx(-1, 1),
        PointIdx(0, 1),
        PointIdx(1, 1)
    };

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

        // Normal calculation vars
        Eigen::Vector3f agregate_n(0.0f,0.0f,0.0f);
        int face_count = 0;

        // Calculate normal from 8 triangles
        int next = 0;
        int first = -1;
        while (true)
        {
            // Find valid indices around 0,0
            int p1idx = -1, p2idx = -1;

            while(next<8){
                if(isValidPoint(pointList[next], width, height, i, cloud)){
                    p1idx = next++;
                    break;
                }
                next++;
            }

            while(next<9){
                // Don't want two of the same indices in different orders
                if(next == 8 && face_count == 1)
                    break;

                // Loop around
                if(next == 8 && face_count > 1){
                    p2idx = first;
                    break;
                }

                int idx = next % 8;
                if(isValidPoint(pointList[idx], width, height, i, cloud)){
                    p2idx = idx;
                    break;
                }
                next++;
            }

            // Stop if no points in range were found
            if(p1idx == -1 || p2idx == -1)
                break;

            if(first == -1)
                first = p1idx;

            PointIdxPair relativePair(pointList[p1idx], pointList[p2idx]);

            // calculate current x and y
            int cur_x = i%width;
            int cur_y = i/width;

            // Calculate absolute indices
            int idx1 = (cur_y+relativePair.p1.y)*width + (cur_x+relativePair.p1.x);
            int idx2 = (cur_y+relativePair.p2.y)*width + (cur_x+relativePair.p2.x);

            pcl::PointXYZI & cp0 = cloud->points[i];
            pcl::PointXYZI & cp1 = cloud->points[idx1];
            pcl::PointXYZI & cp2 = cloud->points[idx2];

            Eigen::Vector3f p0(cp0.x, cp0.y, cp0.z);
            Eigen::Vector3f p1(cp1.x, cp1.y ,cp1.z);
            Eigen::Vector3f p2(cp2.x, cp2.y ,cp2.z);

            // Cross product to get normal
            Eigen::Vector3f vec1 = p1 - p0;
            Eigen::Vector3f vec2 = p2 - p0;
            Eigen::Vector3f tmp = vec1.cross(vec2).normalized();

            // Normalized 0 0 0 == NaN
            if(tmp.x() != tmp.x() ||
               tmp.y() != tmp.y() ||
               tmp.z() != tmp.z()
            )
                continue;

            // Weigh the normals accoring to distance
            /*float avg_dist = (sqrt(vec1.squaredNorm()) + sqrt(vec2.squaredNorm()))/2;
            float x = 100;
            float weight = (x - avg_dist/x);
            agregate_n= agregate_n + weight*tmp;
*/
            agregate_n += tmp;

            face_count++;

        }

        // Face count can be 0 or normals can cancel
        // Should set to point to camera
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

