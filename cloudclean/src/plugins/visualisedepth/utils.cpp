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
        distmap = std::make_shared<std::vector<float>>(size, 0);

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

std::shared_ptr<std::vector<float> > gradientImage(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, int size,
        std::shared_ptr<std::vector<float>> out_image) {

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

void morp_op(float * source, int w, int h, float * dest, int x, int y,
               double * strct, int strct_size, const bool max) {
    assert(filter_size%2 != 0);

    int start = -strct_size/2;
    int end = strct_size/2;

    // Center of stuct is center

    float min_or_max;
    if(max)
        min_or_max = 0;
    else
        min_or_max = FLT_MAX;

    for(int iy = start; iy <= end; iy++){
        for(int ix = start; ix <= end; ix++){
            // map pos
            int _x = ix + x;
            int _y = iy + y;

            // wraps around on edges
            if(_x < 0)
                _x = w+_x;
            else if(_x > w-1)
                _x = _x-w;

            if(_y < 0)
                _y = h+_y;
            else if(_y > h-1)
                _y = _y-h;

            // strct index
            int s = ix+1 + 3*(iy+1);

            // Skip elements not in the structure
            if(strct[s] != 1)
                continue;

            // map index
            int i = _x + w * _y;

            if(max)
                if(source[i] > min_or_max)
                    min_or_max = source[i];
            else if(!max)
                 if(source[i] < min_or_max)
                    min_or_max = source[i];
        }
    }

    dest[x+y*w] = min_or_max;
}
