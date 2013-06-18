#include "plugins/visualisedepth/utils.h"
#include "model/pointcloud.h"
#include "Eigen/Dense"
#include <QDebug>
#include <QTime>
#include <vector>
#include <memory>
#include <pcl/common/pca.h>
#include <pcl/search/flann_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "plugins/visualisedepth/gridsearch.h"

std::shared_ptr<std::vector<int>> makeLookup(std::shared_ptr<PointCloud> cloud) {
    int size = cloud->scan_width() * cloud->scan_height();
    auto grid_to_cloud = std::make_shared<std::vector<int>>(size, -1);
    for(uint i = 0; i < cloud->size(); i++) {
        int grid_idx = cloud->cloudToGridMap()[i];
        grid_to_cloud->at(grid_idx) = i;
    }
    return grid_to_cloud;
}

std::shared_ptr<std::vector<float>> makeDistmap(
        std::shared_ptr<PointCloud> cloud,
        std::shared_ptr<std::vector<float>> distmap) {
    uint size = cloud->scan_width() * cloud->scan_height();

    if(distmap == nullptr || distmap->size() != size)
        distmap = std::make_shared<std::vector<float>>(size, 0.0f);

    float max_dist = 0.0;

    for(uint i = 0; i < cloud->size(); i++) {
        int grid_idx = cloud->cloudToGridMap()[i];
        pcl::PointXYZI & p = cloud->at(i);
        distmap->at(grid_idx) = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

        if(distmap->at(i) > max_dist)
            max_dist = distmap->at(i);
    }

    return distmap;
}

static const double sobel_x[9] = {
    1, 0, -1,
    2, 0, -2,
    1, 0, -1,
};

static const double sobel_y[9] = {
    1, 2, 1,
    0, 0, 0,
    -1, -2, -1,
};

std::shared_ptr<std::vector<float> > gradientImage(std::shared_ptr<std::vector<float>> image,
        int w, int h,
        std::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;

    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    float * grad_mag = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            float gx = convolve_op(w, h, &image->at(0), x, y, sobel_x, 3);
            float gy = convolve_op(w, h, &image->at(0), x, y, sobel_y, 3);
            grad_mag[x+y*w] = sqrt(gx*gx + gy*gy);
        }
    }

    return out_image;
}

std::shared_ptr<std::vector<float> > convolve(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const double * filter, const int filter_size,
        std::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            img[x+y*w] = convolve_op(w, h, &image->at(0), x, y, filter, filter_size);
        }
    }

    return out_image;
}


std::shared_ptr<std::vector<float> > morphology(std::shared_ptr<std::vector<float>> image,
        int w, int h, const int *strct, int strct_size,
        Morphology type,
        std::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            morph_op(&image->at(0), w, h, &out_image->at(0), x, y, strct, strct_size, type);
        }
    }

    return out_image;
}

std::shared_ptr<std::vector<float> > stdev(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const int local_size,
        std::shared_ptr<std::vector<float>> out_image) {

    int size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            img[x+y*w] = stdev_op(w, h, &image->at(0), x, y, local_size);
        }
    }

    return out_image;
}

std::shared_ptr<std::vector<float> > interpolate(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const int nsize,
        std::shared_ptr<std::vector<float>> out_image) {

    int size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            interp_op(&image->at(0), w, h, &out_image->at(0), x, y, nsize);
        }
    }

    return out_image;
}

std::shared_ptr<std::vector<float> > stdev_depth(std::shared_ptr<PointCloud> cloud, const double radius) {
    std::shared_ptr<std::vector<float>> stdevs = std::make_shared<std::vector<float>>(cloud->size());

    std::vector<int> idxs(0);
    std::vector<float> dists(0);

    // 1m radius
    //const double radius = 1.0;

    GridSearch gs(*cloud);

    // center
    Eigen::Map<Eigen::Vector3f> center(cloud->sensor_origin_.data());

    const Octree::Ptr ot = cloud->octree();
    for(uint i = 0; i < cloud->size(); i++){
        idxs.clear();
        dists.clear();
        //ot->radiusSearch(cloud->points[i], radius, idxs, dists);
        //grid_nn_op(i, *cloud, idxs, 1, 50);
        gs.radiusSearch(cloud->at(i), radius, idxs, dists, 50);

        // calculate stdev of the distances?
        // bad idea because you have a fixed radius
        // Calculate distance from center of scan

        Eigen::Map<const Eigen::Vector3f> query_point(&(cloud->points[i].x));

        float sum = 0.0f;
        float sum_sq = 0.0f;

        for(int idx : idxs) {
            const float * data = &(cloud->points[idx].x);
            Eigen::Map<const Eigen::Vector3f> point(data);
            float dist = (point-center).norm();
            //float dist = (point-query_point).norm();
            sum += dist;
            sum_sq += dist*dist;
        }

        if(idxs.size() > 2)
            (*stdevs)[i] = (sum_sq - (sum*sum)/(idxs.size()))/((idxs.size())-1);
        else
            (*stdevs)[i] = 0;

    }

    return stdevs;
}


std::shared_ptr<std::vector<float>> cloudToGrid(const std::vector<int> &map,
        int img_size,
        std::shared_ptr<std::vector<float>> input,
        std::shared_ptr<std::vector<float>> img) {

    if(img == nullptr || img->size() != img_size)
        img = std::make_shared<std::vector<float>>(img_size, 0.0f);

    for(uint i = 0; i < map.size(); i++) {
        int grid_idx = map[i];
        (*img)[grid_idx] = (*input)[i];
    }

    return img;
}

std::shared_ptr<std::vector<Eigen::Vector3f> > getPCA(std::shared_ptr<PointCloud> cloud, double radius, int max_nn) {

    std::shared_ptr<std::vector<Eigen::Vector3f> > eigen_vals =
            std::make_shared<std::vector<Eigen::Vector3f>>(cloud->size());

    //GridSearch search(*cloud);
    //pcl::search::FlannSearch<pcl::PointXYZI> search;
    pcl::KdTreeFLANN<pcl::PointXYZI> search;
    search.setInputCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr(cloud.get(), boost::serialization::null_deleter()));


    QTime t;
    t.start();

    int less_than_three_points_count = 0;

    boost::shared_ptr <std::vector<int> > kIdxs;
    kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
    std::vector<float> kDist;

    // For every point
    for(unsigned int i = 0; i < cloud->size(); i++){
        if(i < 10) {
            qDebug() << "Loop: " << i;
        }


        if(i % 2000 == 0) {
            int ms = t.restart();
            qDebug() << "so " << less_than_three_points_count << "out of " << i << "points have less than 2 neighbours";
            qDebug() << "Radius: " << radius << "Max nn: " << max_nn;
            qDebug() << "% done: " << float(i) / cloud->size();
            qDebug() << "MS per loop" << float(ms)/2000.0f;
        }

        search.radiusSearch(i, radius, *kIdxs, kDist, max_nn);

        if(kIdxs->size() > max_nn){
            qDebug() << "Whoops! Too many";
            continue;
        }

        if(kIdxs->size() < 3) {
            less_than_three_points_count++;
            (*eigen_vals)[i] = Eigen::Vector3f(0, 0, 1.0f); // Assume isotaled point
            continue;
        }

        pcl::PCA<pcl::PointXYZI> pcEstimator(true);
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr const_cloud(cloud.get(), boost::serialization::null_deleter());
        pcEstimator.setInputCloud (const_cloud);
        pcEstimator.setIndices(kIdxs);
        eigen_vals->at(i) = pcEstimator.getEigenValues();

        // Sort and normalise
        float * eigenvalues = eigen_vals->at(i).data();

        for(int j = 0; j < 3; j++ ){
            int max = j;
            for(int k = j; k < 3; k++ ){
                if(eigenvalues[k] > eigenvalues[max])
                    max = k;
            }
            float tmp = eigenvalues[j];
            eigenvalues[j] = eigenvalues[max];
            eigenvalues[max] = tmp;
        }

        eigen_vals->at(i).normalize(); // SHOULD THIS BE NORMALISED?
    }

    return eigen_vals;
}