#include "plugins/visualisedepth/utils.h"
#include "model/pointcloud.h"
#include <QDebug>

std::shared_ptr<std::vector<int>> makeLookup(std::shared_ptr<PointCloud> cloud) {
    int size = cloud->scan_width_ * cloud->scan_height_;
    auto grid_to_cloud = std::make_shared<std::vector<int>>(size, -1);
    for(int i = 0; i < cloud->size(); i++) {
        int grid_idx = cloud->cloud_to_grid_map_[i];
        grid_to_cloud->at(grid_idx) = i;
    }
    return grid_to_cloud;
}

std::shared_ptr<std::vector<float>> makeDistmap(
        std::shared_ptr<PointCloud> cloud,
        std::shared_ptr<std::vector<float>> distmap) {
    int size = cloud->scan_width_ * cloud->scan_height_;

    if(distmap == nullptr || distmap->size() != size)
        distmap = std::make_shared<std::vector<float>>(size, 0.0f);

    float max_dist = 0.0;

    for(int i = 0; i < cloud->size(); i++) {
        int grid_idx = cloud->cloud_to_grid_map_[i];
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

    int size = w*h;

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

    int size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = std::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    // Calculate the gradient magnitude
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

    int size = w*h;
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
