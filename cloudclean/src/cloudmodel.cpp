#include "cloudmodel.h"
#include <ctime>
#include "io.h"
#include <pcl/filters/filter.h>
#include <QDebug>

CloudModel* CloudModel::only_instance = NULL;

CloudModel::CloudModel(QObject *parent) :
    QObject(parent), point_buffer( QGLBuffer::VertexBuffer )
{
    //normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    //kdtree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    //fpfhs = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);

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
        if(!layers[l].active)
            continue;
        layers[l].copyFromGPU();
        for(unsigned int i = 0; i < layers[l].index.size(); i++){
            printf("i: %d\n", i);
            int idx = layers[l].index[i];
            if(idx == -1)
                continue;
            original_cloud->points[cloud_to_grid_map[idx]] = cloud->points[idx];
        }
    }


    save_ptx(output_file, original_cloud);
    return true;
}

bool CloudModel::createBuffers(){

    //printf("Cloud size: %d\n", cloud->size());

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

    layer.colour = Eigen::Vector3f(1.0f, 0.5f, 0.5f);
    layer.gl_index_buffer.create();
    layer.gl_index_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    layer.gl_index_buffer.bind();
    layer.gl_index_buffer.allocate(cloud->size() * sizeof(int) );

    /// Initialise the first layer to include all points
    for(unsigned int i = 0; i < cloud->size(); i++){
        layer.gl_index_buffer.write(i*sizeof(int), reinterpret_cast<const void *>(&i), sizeof(int));
    }

    printf("Buffers created!\n");
}

bool CloudModel::loadFile(const char * input_file, int subsample){

    printf("File: %s\n", input_file);
    printf("Subsample: %d\n", subsample);

    // Time code
    time_t f_begin, f_end;
    //time_t n_begin, n_end;
    //time_t fpfh_begin, fpfh_end;

    time(&f_begin); // Timing
    cloud = read_ptx(input_file, subsample);
    time(&f_end); // Timing

    x_dim = cloud->width;
    y_dim = cloud->height;

    /// Filter and flatten point cloud
    pcl::removeNaNFromPointCloud(*cloud, *cloud, cloud_to_grid_map);

    loaded = true;
    createBuffers();

    //layerList.reset();

    /*

    time(&n_begin); // Timing


    // Estimate normals
    normal_estimation(cloud, normals);

    time(&n_end); // Timing

    vals_in_range = 15;
    K = 20;
    radius = 0.05f;

    time(&fpfh_begin); // Timing

    // Calulate the FPFH:

    fpfhs = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);

    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);
    fpfh.setSearchMethod (tree);
    fpfh.setKSearch (K);
    //fpfh.setRadiusSearch (radius);
    fpfh.setIndices(p_valid_indices);
    fpfh.compute (*fpfhs);

    time(&fpfh_end); // Timing

    kdtree->setInputCloud (cloud);


    printf("File read: %f\n", difftime(f_end, f_begin));
    printf("Normal estimation: %f\n", difftime(n_end, n_begin));
    printf("FPFH %f\n", difftime(fpfh_end, fpfh_begin));
*/
    return true;
}

bool CloudModel::isLoaded(){
    return loaded;
}

void normal_estimation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
    int length = cloud->points.size();
    int width = cloud->width;
    int height = cloud->height;
    normals->points.resize(length);

    for (int i = 0; i < length; ++i)
    {
        if (cloud->points[i].x != cloud->points[i].x)
        {
            normals->points[i].data_n[0] = NAN;
            normals->points[i].data_n[1] = NAN;
            normals->points[i].data_n[2] = NAN;
            normals->points[i].data_n[3] = NAN;
            continue;
        }

        // relavite indices
        int idx[16] = {-1,-1, 0,-1, 1,-1, 1,0, 1,1, 0,1, -1,1, -1,0};


        // Normal calculation vars
        Eigen::Vector3f agregate_n(0.0f,0.0f,0.0f);
        int count_n = 0;

        // previous index
        int p1x = -1, p1y = 0;
        int p2x = -1, p2y = 0;

        int idx1, idx2;

        // Calculate normal fron 8 triangles
        for (int j = 0; j < 8; ++j)
        {
            // relative index
            p2x = idx[j*2];
            p2y = idx[j*2+1]; // Update p2 index

            // abs index
            idx1 = i + width*p1y + p1x;
            idx2 = i + width*p2y + p2x;


            if ( // Not out of bounds
                    (!(
                        (i%width == 0 && (p2x == 1 || p1x == 1) ) || // right overun
                        (i%width == 1 && (p2x == -1 || p1x == -1) ) || // left overun
                        (i <= width && (p2y == -1 || p1y == -1) ) || // top overun
                        (i/width == (height-1) && (p2y == 1 || p1y == 1) ) // bottom overun TODO:Check if this holds
                    )) &&
                    // No NaNs
                    (
                        (cloud->points[idx1].x == cloud->points[idx1].x) && // invalid point 1
                        (cloud->points[idx2].x == cloud->points[idx2].x)	// invalid point 2
                    )
            )
            {

                Eigen::Vector3f p0(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                Eigen::Vector3f p1(cloud->points[idx1].x, cloud->points[idx1].y ,cloud->points[idx1].z);
                Eigen::Vector3f p2(cloud->points[idx2].x, cloud->points[idx2].y ,cloud->points[idx2].z);

                Eigen::Vector3f tmp = ((p1 - p0).cross(p2 - p0));

                agregate_n= agregate_n + tmp.normalized();
                count_n++;

                }
                p1x = p2x; p1y = p2y; // make p1 current p2

        }

        if(count_n == 0){
                count_n = 1;
        }

        agregate_n = (agregate_n/count_n).normalized();

        Eigen::Vector4f normal(agregate_n(0), agregate_n(1), agregate_n(2),0.0f);

        pcl::flipNormalTowardsViewpoint (cloud->points[i], cloud->sensor_origin_[0], cloud->sensor_origin_[1], cloud->sensor_origin_[2],
                agregate_n(0), agregate_n(1), agregate_n(2));

        normals->points[i].data_n[0] = agregate_n(0);
        normals->points[i].data_n[1] = agregate_n(1);
        normals->points[i].data_n[2] = agregate_n(2);
        normals->points[i].data_n[3] = 0.0f;

    }
}
