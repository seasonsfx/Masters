#include "utilities/cv.h"
#include "model/pointcloud.h"
#include <cfloat>
#include <Eigen/Dense>
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
        (*grid_to_cloud)[grid_idx] = i;
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
        pcl::PointXYZI & p = (*cloud)[i];
        (*distmap)[grid_idx] = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

        if((*distmap)[i] > max_dist)
            max_dist = (*distmap)[i];
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
            float gx = convolve_op(w, h, &(*image)[0], x, y, sobel_x, 3);
            float gy = convolve_op(w, h, &(*image)[0], x, y, sobel_y, 3);
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
            img[x+y*w] = convolve_op(w, h, &(*image)[0], x, y, filter, filter_size);
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

    //float * img = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            morph_op(&(*image)[0], w, h, &(*out_image)[0], x, y, strct, strct_size, type);
        }
    }

    return out_image;
}

// stdev on depth image
std::shared_ptr<std::vector<float> > stdev(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const int local_size,
        std::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            img[x+y*w] = stdev_op(w, h, &(*image)[0], x, y, local_size);
        }
    }

    return out_image;
}

std::shared_ptr<std::vector<float> > interpolate(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const int nsize,
        std::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    //float * img = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            interp_op(&(*image)[0], w, h, &(*out_image)[0], x, y, nsize);
        }
    }

    return out_image;
}

std::shared_ptr<std::vector<float> > stdev_dist(std::shared_ptr<PointCloud> cloud,
                            const double radius, int max_nn, bool use_depth) {

    std::shared_ptr<std::vector<float>> stdevs
                        = std::make_shared<std::vector<float>>(cloud->size());

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
        gs.radiusSearch((*cloud)[i], radius, idxs, dists, max_nn);

        // calculate stdev of the distances?
        // bad idea because you have a fixed radius
        // Calculate distance from center of scan

        Eigen::Map<const Eigen::Vector3f> query_point(&(cloud->points[i].x));

        float sum = 0.0f;
        float sum_sq = 0.0f;

        for(int idx : idxs) {
            const float * data = &(cloud->points[idx].x);
            Eigen::Map<const Eigen::Vector3f> point(data);

            float dist;

            if(use_depth)
                dist = (point-center).norm();
            else
                dist = (point-query_point).norm();

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
        uint img_size,
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

std::shared_ptr<std::vector<Eigen::Vector3f> > getHist(std::shared_ptr<PointCloud> cloud, double radius, uint max_nn) {

    std::shared_ptr<std::vector<Eigen::Vector3f> > eigen_vals =
            std::make_shared<std::vector<Eigen::Vector3f>>(cloud->size());

    pcl::KdTreeFLANN<pcl::PointXYZI> search;
    search.setInputCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr(cloud.get(), boost::serialization::null_deleter()));


    QTime total;
    total.start();

    int less_than_three_points_count = 0;

    boost::shared_ptr <std::vector<int> > kIdxs;
    kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
    std::vector<float> kDist;

    float min = FLT_MAX;
    float max = FLT_MIN;

    // For every point
    for(unsigned int i = 0; i < cloud->size(); i++){

        search.radiusSearch(i, radius, *kIdxs, kDist, max_nn);

        if(kIdxs->size() > max_nn){
            qDebug() << "Whoops! Too many";
            continue;
        }

        if(kIdxs->size() < 3) {
            less_than_three_points_count++;
            (*eigen_vals)[i] = Eigen::Vector3f(0, 0, 1.0f); // Assume isolated point
            continue;
        }

        pcl::PCA<pcl::PointXYZI> pcEstimator(true);
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr const_cloud(cloud.get(), boost::serialization::null_deleter());
        pcEstimator.setInputCloud (const_cloud);
        pcEstimator.setIndices(kIdxs);
        (*eigen_vals)[i] = pcEstimator.getEigenValues();


        (*eigen_vals)[i].normalize(); // SHOULD THIS BE NORMALISED?

    }

    /*
    for(unsigned int i = 0; i < cloud->size(); i++){
        (*eigen_vals)[i] = ((*eigen_vals)[i] - Eigen::Vector3f(min, min, min) ) / (max - min);
    }
    */

    qDebug() << "Radius: " << radius << " Max_nn: " << max_nn << " Time: " << total.elapsed()/1000.0f << "Sec";
    qDebug("Points with less than %d neighbours: %d", max_nn, less_than_three_points_count);
    qDebug("Max: %f, Min: %f", max, min);

    return eigen_vals;
}

std::shared_ptr<std::vector<Eigen::Vector3f> > getPCA(pcl::PointCloud<pcl::PointXYZI> * cloud, double radius, uint max_nn) {

    std::shared_ptr<std::vector<Eigen::Vector3f> > eigen_vals =
            std::make_shared<std::vector<Eigen::Vector3f>>(cloud->size());

    //GridSearch search(*cloud);
    //pcl::search::FlannSearch<pcl::PointXYZI> search;
    pcl::KdTreeFLANN<pcl::PointXYZI> search;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cptr(cloud, boost::serialization::null_deleter());
    search.setInputCloud(cptr);


    QTime total;
    total.start();

    QTime t;
    t.start();

    int less_than_three_points_count = 0;

    boost::shared_ptr <std::vector<int> > kIdxs;
    kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
    std::vector<float> kDist;

    float min = FLT_MAX;
    float max = FLT_MIN;

    // For every point
    for(unsigned int i = 0; i < cloud->size(); i++){

        if(i % 200000 == 0) {
            int ms = t.restart();
            qDebug() << "so " << less_than_three_points_count << "out of " << i << "points have less than 2 neighbours";
            qDebug() << "Radius: " << radius << "Max nn: " << max_nn;
            qDebug() << "% done: " << float(i) / cloud->size();
            qDebug() << "MS per loop" << float(ms)/200000.0f;
        }

        search.radiusSearch(i, radius, *kIdxs, kDist, max_nn);

        if(kIdxs->size() > max_nn && max_nn > 0){
            qDebug() << "Whoops! Too many";
            continue;
        }

        if(kIdxs->size() < 3) {
            less_than_three_points_count++;
            (*eigen_vals)[i] = Eigen::Vector3f(0, 0, 1.0f); // Assume isolated point
            continue;
        }

        pcl::PCA<pcl::PointXYZI> pcEstimator(true);
        pcEstimator.setInputCloud (cptr);
        pcEstimator.setIndices(kIdxs);
        (*eigen_vals)[i] = pcEstimator.getEigenValues();

/*
        for(int j = 0; j < 3; j++ ){
            float val = (*eigen_vals)[i][j];
            if(val > max)
                max = val;
            else if(val < min)
                min = val;
        }
*/
        (*eigen_vals)[i].normalize(); // SHOULD THIS BE NORMALISED?

    }

    /*
    for(unsigned int i = 0; i < cloud->size(); i++){
        (*eigen_vals)[i] = ((*eigen_vals)[i] - Eigen::Vector3f(min, min, min) ) / (max - min);
    }
    */

    qDebug() << "Radius: " << radius << " Max_nn: " << max_nn << " Time: " << total.elapsed()/1000.0f << "Sec";
    qDebug("Points with less than %d neighbours: %d", max_nn, less_than_three_points_count);
    qDebug("Max: %f, Min: %f", max, min);

    return eigen_vals;
}

std::shared_ptr<std::vector<float> > normal_stdev(std::shared_ptr<PointCloud> cloud,
                  pcl::PointCloud<pcl::Normal>::Ptr normals,
                  double radius, int max_nn) {

    std::shared_ptr<std::vector<float> > std_devs =
            std::make_shared<std::vector<float>>(cloud->size(), 0);

    pcl::KdTreeFLANN<pcl::PointXYZI> search;
    search.setInputCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr(cloud.get(), boost::serialization::null_deleter()));


    QTime t;
    t.start();

    int less_than_three_points_count = 0;

    boost::shared_ptr <std::vector<int> > kIdxs;
    kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>(cloud->size(), 0));
    std::vector<float> kDist;

    // For every point
    for(unsigned int i = 0; i < cloud->size(); i++){

        if(i % 20000 == 0) {
            int ms = t.restart();
            qDebug() << "so " << less_than_three_points_count << "out of " << i << "points have less than 2 neighbours";
            qDebug() << "Radius: " << radius << "Max nn: " << max_nn;
            qDebug() << "% done: " << float(i) / cloud->size();
            qDebug() << "MS per loop: " << float(ms)/20000.0f;
        }

        // TODO(Rickert): Sort out Nan in formal estimation
        // Skip NAN's
        if((*normals)[i].data_n[0] != (*normals)[i].data_n[0])
            continue;

        search.radiusSearch(i, radius, *kIdxs, kDist, max_nn);

        if(kDist.size() < 3) {
            less_than_three_points_count++;
            continue;
        }

        std::vector<float> angles;
        angles.resize(kDist.size());

        Eigen::Map<Eigen::Vector3f> current((*normals)[i].data_n);

        float sumOfSquares = 0.0f;
        float sum = 0.0f;

        for(int idx : *kIdxs) {

            Eigen::Map<Eigen::Vector3f> neighbour((*normals)[idx].data_n);

            float cosine = neighbour.dot(current) /
                    neighbour.norm()*current.norm();

            cosine = clamp(cosine, 0.0f, 1.0f);

            // Normalised angle
            float angle = acos(cosine)/M_PI;

            sum += angle;
            sumOfSquares += angle*angle;
        }

        float std_dev = sqrt( (sumOfSquares/angles.size()) - pow(sum/angles.size(), 2));

        if(std_dev != std_dev) {
            //qDebug() << "Bugger! NAN";
            continue;
        }


        (*std_devs)[i] = std_dev;

    }

    return std_devs;
}
