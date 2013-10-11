#ifndef ULTILITIES_FILTERS_H
#define ULTILITIES_FILTERS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_container.h>
#include <boost/serialization/shared_ptr.hpp>

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr octreeDownsample(
        pcl::PointCloud<PointT> * input,
        float resolution,
        std::vector<int> & sub_idxs) {

    size_t data_items = sizeof(PointT)/sizeof(float);

    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>());
    sub_idxs.resize(input->size(), 0);

    typename pcl::PointCloud<PointT>::Ptr ptr(input, boost::serialization::null_deleter());

    typename pcl::octree::OctreePointCloud<PointT> octree1(resolution);
    octree1.setInputCloud (ptr);
    octree1.addPointsFromInputCloud();

    typename pcl::octree::OctreePointCloud<PointT>::LeafNodeIterator it1;
    typename pcl::octree::OctreePointCloud<PointT>::LeafNodeIterator it1_end = octree1.leaf_end();

    unsigned int leafNodeCounter = 0;

    for (it1 = octree1.leaf_begin(); it1 != it1_end; ++it1) {
        std::vector<int> & indices = it1.getLeafContainer().getPointIndicesVector();

        PointT p;
        Eigen::Map<Eigen::VectorXf> pmap = Eigen::VectorXf::Map(&p.data[0], data_items);

        for(int idx : indices){


            Eigen::Map<Eigen::VectorXf> pmap1 = Eigen::VectorXf::Map(reinterpret_cast<float *>(&((*input)[idx])), data_items);
            pmap += pmap1;
            sub_idxs[idx] = output->size();

        }

        float size_inv = 1.0/indices.size();

        pmap*=size_inv;
        output->push_back(p);

        leafNodeCounter++;
    }

    return output;
}


template <typename T, typename T2>
void map(std::vector<T, T2> & small, std::vector<T, T2>& big, std::vector<int> & map){

    big.resize(map.size());
    for(uint i = 0; i < map.size(); i++) {
        int small_idx = map[i];

        memcpy(&big[i], &small[small_idx], sizeof(T));
    }
}

#endif  // ULTILITIES_FILTERS_H
