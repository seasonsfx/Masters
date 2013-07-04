
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
