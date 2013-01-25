/*
 * Software License Agreement (BSD License)
 *
 *  CloudClean
 *  Copyright (c) 2013, Rickert Mulder
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
 *   * Neither the name of Rickert Mulder nor the names of its
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

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_POINTPICKER_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_POINTPICKER_H_

#include <Eigen/Dense>
#include <pcl/octree/octree.h>
#include "cloudclean/cloudclean_global.h"

class GLArea;
class CloudModel;

class DLLSPEC PointPicker {
 public:
    PointPicker(GLArea * glarea = nullptr, CloudModel *cm = nullptr,
                float max_dist = 0);
    int pick(int x, int y, int source_layer);

 private:
    void getClickRay(int x, int y, Eigen::Vector3f& p1, Eigen::Vector3f& p2);

 private:
    GLArea * glarea;
    CloudModel *cm;
    float max_dist;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr  octree;
};

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_POINTPICKER_H_