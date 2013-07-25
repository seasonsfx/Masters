#ifndef VISIUALISE_DEPTH_UTIL
#define VISIUALISE_DEPTH_UTIL

#include <memory>
#include <vector>
#include <cassert>
#include "model/pointcloud.h"

std::shared_ptr<std::vector<float> > test_feature(std::shared_ptr<PointCloud> cloud,
                                 const double radius, int max_nn = 0, bool use_depth = false);

#endif  // VISIUALISE_DEPTH_UTIL
