#include "appdata.h"
#include <ctime>
#include "io.h"

AppData* AppData::only_instance = NULL;

AppData::AppData(QObject *parent) :
    QObject(parent)
{
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    kdtree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    fpfhs = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
}

AppData * AppData::Instance(){
    if(!only_instance)
        only_instance = new AppData();
    return only_instance;
    
    
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
                    !(
                        (i%width == 0 && (p2x == 1 || p1x == 1) ) || // right overun
                        (i%width == 1 && (p2x == -1 || p1x == -1) ) || // left overun
                        (i/width == 0 && (p2y == -1 || p1y == -1) ) || // top overun
                        (i/width == (height-1) && (p2y == 1 || p1y == 1) ) // bottom overun
                    ) &&
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

bool AppData::loadFile(const char * input_file, int subsample){

    printf("File: %s\n", input_file);
    printf("Subsample: %d\n", subsample);

    // Time code
    time_t f_begin, f_end;
    time_t n_begin, n_end;
    time_t fpfh_begin, fpfh_end;

    time(&f_begin); // Timing
    cloud = read_ptx(input_file, subsample);
    time(&f_end); // Timing

    int point_count = cloud->points.size();
    invalid_points = 0;

    // Mark valid points
    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> ());
    for (int i = 0; i < point_count; ++i)
    {
        if (cloud->points[i].x == cloud->points[i].x)
        {
            indicesptr->push_back(i);
        }
        else
            invalid_points++;
    }

    printf("Points: %d\n", point_count);
    printf("Invalid points: %d (%f %%) \n", invalid_points, invalid_points/(float)point_count);

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
    fpfh.setIndices(indicesptr);
    fpfh.compute (*fpfhs);

    time(&fpfh_end); // Timing

    kdtree->setInputCloud (cloud);


    printf("File read: %f\n", difftime(f_end, f_begin));
    printf("Normal estimation: %f\n", difftime(n_end, n_begin));
    printf("FPFH %f\n", difftime(fpfh_end, fpfh_begin));

    return true;
}
