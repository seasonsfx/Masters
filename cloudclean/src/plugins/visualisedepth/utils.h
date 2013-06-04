#ifndef VISIUALISE_DEPTH_UTIL
#define VISIUALISE_DEPTH_UTIL

#include <memory>
#include <vector>
#include <cassert>
#include <limits>
#include <QDebug>
class PointCloud;

template <typename T>
void minmax(std::vector<T> & v, T & min, T & max){
    min = std::numeric_limits<T>::max();
    max = std::numeric_limits<T>::min();
    for(auto val : v){
        if(val > max)
            max = val;
        else if(val < min)
            min = val;
    }

}

std::shared_ptr<std::vector<int>> makeLookup(
        std::shared_ptr<PointCloud> cloud
);

std::shared_ptr<std::vector<float>> makeDistmap(
        std::shared_ptr<PointCloud> cloud,
        std::shared_ptr<std::vector<float>> distmap = nullptr
);

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
            int f = ix+1 + 3*(iy+1);

            sum += source[i] * filter[f];
        }
    }
    return sum;
}


std::shared_ptr<std::vector<float> > gradientImage(std::shared_ptr<std::vector<float>> image,
        int w, int h, int size,
        std::shared_ptr<std::vector<float>> out_image = nullptr);

std::shared_ptr<std::vector<float> > convolve(
        std::shared_ptr<std::vector<float>> image,
        int w, int h, const double * filter, const int filter_size,
        std::shared_ptr<std::vector<float>> out_image = nullptr);

#endif  // VISIUALISE_DEPTH_UTIL
