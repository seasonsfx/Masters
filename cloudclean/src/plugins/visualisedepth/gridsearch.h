/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef GRID_SEARCH_H_
#define GRID_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/for_each_type.h>
#include <pcl/common/concatenate.h>
#include <pcl/search/search.h>


/** \brief Generic search class. All search wrappers must inherit from this.
  *
  * Each search method must implement 2 different types of search:
  *   - \b nearestKSearch - search for K-nearest neighbors.
  *   - \b radiusSearch - search for all nearest neighbors in a sphere of a given radius
  *
  * The input to each search method can be given in 3 different ways:
  *   - as a query point
  *   - as a (cloud, index) pair
  *   - as an index
  *
  * For the latter option, it is assumed that the user specified the input
  * via a \ref setInputCloud () method first.
  *
  * \note In case of an error, all methods are supposed to return 0 as the number of neighbors found.
  *
  * \note libpcl_search deals with three-dimensional search problems. For higher
  * level dimensional search, please refer to the libpcl_kdtree module.
  *
  * \author Radu B. Rusu
  * \ingroup search
  */
template<typename PointT>
class GridSearch : public pcl::search::Search
{
  public:

    /** Constructor. */
    //GridSearch (const std::string& name = "", bool sorted = false);

    /** Destructor. */
    virtual
    ~GridSearch ()
    {
    }


    /** \brief Search for the k-nearest neighbors for the given query point.
      * \param[in] point the given query point
      * \param[in] k the number of neighbors to search for
      * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
      * a priori!)
      * \return number of neighbors found
      */
    virtual int
    nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices,
                    std::vector<float> &k_sqr_distances) const = 0;


    /** \brief Search for all the nearest neighbors of the query point in a given radius.
      * \param[in] point the given query point
      * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
      * \param[out] k_indices the resultant indices of the neighboring points
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
      * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
      * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
      * returned.
      * \return number of neighbors found in radius
      */
    virtual int
    radiusSearch (const PointT& point, double radius, std::vector<int>& k_indices,
                  std::vector<float>& k_sqr_distances, unsigned int max_nn = 0) const = 0;


  protected:

/*
    PointCloudConstPtr input_;
    IndicesConstPtr indices_;
    bool sorted_results_;
    std::string name_;
*/

}; // class GridSearch

#endif  //#ifndef GRID_SEARCH_H_
