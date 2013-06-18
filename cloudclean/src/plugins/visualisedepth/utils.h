#ifndef VISIUALISE_DEPTH_UTIL
#define VISIUALISE_DEPTH_UTIL

#include <memory>
#include <vector>
#include <cassert>
#include <limits>
#include <QDebug>
#include "model/pointcloud.h"

enum class Morphology{ERODE, DILATE};

std::shared_ptr<std::vector<float>> makeDistmap(
        std::shared_ptr<PointCloud> cloud,
        std::shared_ptr<std::vector<float>> distmap = nullptr
);

std::shared_ptr<std::vector<float> > gradientImage(std::shared_ptr<std::vector<float>> image,
        int w, int h,
        std::shared_ptr<std::vector<float>> out_image = nullptr);

std::shared_ptr<std::vector<float> > convolve(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const double * filter, const int filter_size,
        std::shared_ptr<std::vector<float>> out_image = nullptr);

std::shared_ptr<std::vector<float> > morphology(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const int * strct, int strct_size,
        Morphology type,
        std::shared_ptr<std::vector<float>> out_image = nullptr);

std::shared_ptr<std::vector<float> > stdev(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const int local_size,
        std::shared_ptr<std::vector<float>> out_image = nullptr);

template <typename T>
void minmax(const std::vector<T> & v, T & min, T & max){
    min = std::numeric_limits<T>::max();
    max = std::numeric_limits<T>::min();
    for(auto val : v){
        if(val > max)
            max = val;
        else if(val < min)
            min = val;
    }

}

std::shared_ptr<std::vector<float> > interpolate(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const int nsize,
        std::shared_ptr<std::vector<float>> out_image = nullptr);

std::shared_ptr<std::vector<float> > stdev_depth(std::shared_ptr<PointCloud> cloud, const double radius);

std::shared_ptr<std::vector<float>> cloudToGrid(const std::vector<int> & map, int img_size,
        std::shared_ptr<std::vector<float>> input,
        std::shared_ptr<std::vector<float>> img = nullptr);

std::shared_ptr<std::vector<Eigen::Vector3f> > getPCA(std::shared_ptr<PointCloud> cloud, double radius, int max_nn = 0);


/// Inline functions:

inline float convolve_op(
        int w, int h, float * source, int x, int y,
        const double * filter, int filter_size) {
    assert(filter_size%2 != 0);

    int start = -filter_size/2;
    int end = filter_size/2;

    float sum = 0.0f;

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

            // map index
            int i = _x + w * _y;
            // filter index
            int f = ix+end + filter_size*(iy+end);

            float val = source[i] * filter[f];

            sum += val;
        }
    }
    return sum;
}

inline void morph_op(float * source, int w, int h, float * dest, int x, int y,
               const int * strct, int strct_size, const Morphology type) {
    assert(strct_size%2 != 0);

    int start = -strct_size/2;
    int end = strct_size/2;

    // Center of stuct is center

    float min_or_max;
    if(type == Morphology::DILATE)
        min_or_max = 0;
    else
        min_or_max = std::numeric_limits<float>::max();

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

            if(type == Morphology::DILATE){
                if(source[i] > min_or_max)
                    min_or_max = source[i];
            }
            else {
                 if(source[i] < min_or_max)
                    min_or_max = source[i];
            }
        }
    }


    dest[x+y*w] = min_or_max;
}

inline float stdev_op(
        int w, int h, float * source, int x, int y, int size) {
    assert(size%2 != 0);

    int start = -size/2;
    int end = size/2;

    float sum = 0.0f;
    float sum_sq = 0.0f;

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

            // map index
            int i = _x + w * _y;

            sum += source[i];
            sum_sq += source[i] * source[i];;
        }
    }
    return (sum_sq - (sum*sum)/(w*h))/((w*h)-1);
}

inline void interp_op(float * source, int w, int h, float * dest, int x, int y,
               int nsize) {
    assert(nsize%2 != 0);

    int start = -nsize/2;
    int end = nsize/2;

    // Dont need to interopolate
    if(source[x+y*w] > 1e-6){
        dest[x+y*w] = source[x+y*w];
        return;
    }

    float sum = 0.0f;
    int n = 0;

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

            // source index
            int i = _x + w * _y;

            // Skip null values
            if(source[i]  < 1e-6)
                continue;

            sum += source[i];
            n++;
        }
    }

    if(n == 0){
        dest[x+y*w] = source[x+y*w];
        return;
    }

    dest[x+y*w] = sum/n;
}


inline void grid_nn_op(int idx,
                       PointCloud & cloud,
                       std::vector<int> & idxs,
                       double radius,
                       int max_nn) {


    int h = cloud.scan_height();
    int w = cloud.scan_width();

    std::shared_ptr<const std::vector<int>> grid_to_cloud = cloud.gridToCloudMap();

    int grid_idx = cloud.cloudToGridMap()[idx];
    int x = grid_idx / h;
    int y = grid_idx % h;

    Eigen::Map<Eigen::Vector3f> query_point(&cloud.points[idx].x, 3);

    const double rad_sq = radius * radius;
    const int max_ring_size = (w>h?w:h)/2;
    int outside_radius = 0;

    for(int ring = 1; ring <= max_ring_size ; ring++){
        //qDebug("Ring %d", ring);
        // Iterator over edge of square
        for(int iy = -ring; iy <= ring; iy++){
            for(int ix = -ring; ix <=ring; ix++){

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


                // source index
                int i = _y + h * _x;
                int idx = (*grid_to_cloud)[i];

                // Only look at valid indexes
                if(idx  != -1){
                    float * data = &(cloud.points[idx].x);
                    Eigen::Map<Eigen::Vector3f> neighbour(data, 3);
                    float sqdist = (neighbour-query_point).squaredNorm();

                    if(sqdist <= rad_sq) {
                        idxs.push_back(idx);
                        if(idxs.size() > max_nn)
                            return;
                    } else {
                        outside_radius++;
                    }

                    int side_len = (ring*2+1);
                    int max_err = side_len*2 + 2*(side_len-2);

                    // If error is more than the indexes in a ring
                    if(outside_radius > max_err){
                        //qDebug() << "Exeeded" << max_err;
                        return;
                    }
                }


                // Skip the inner values
                if(iy != -ring && iy != ring && ix == -ring) {
                    ix = ring-1;
                }
            }
        }
    }

}


static const double gaussian[25] = {
    0.00296901674395065, 0.013306209891014005, 0.02193823127971504, 0.013306209891014005, 0.00296901674395065,
    0.013306209891014005, 0.05963429543618023, 0.09832033134884507, 0.05963429543618023, 0.013306209891014005,
    0.02193823127971504, 0.09832033134884507, 0.16210282163712417, 0.09832033134884507, 0.02193823127971504,
    0.013306209891014005, 0.05963429543618023, 0.09832033134884507, 0.05963429543618023, 0.013306209891014005,
    0.00296901674395065, 0.013306209891014005, 0.02193823127971504, 0.013306209891014005, 0.00296901674395065,
};

#endif  // VISIUALISE_DEPTH_UTIL
