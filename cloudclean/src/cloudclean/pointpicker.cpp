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

#include <cmath>
#include <cfloat>
#include <vector>

#include "glarea.h"
#include "cloudmodel.h"
#include "pointpicker.h"

#include <GL/glu.h>

PointPicker::PointPicker(GLArea *glarea, CloudModel *cm, float max_dist):
        glarea(glarea), cm(cm), max_dist(max_dist){

    double resolution = 0.2;
    octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr(
                new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(
                    resolution));
    octree->setInputCloud(cm->cloud);
    octree->defineBoundingBox();
    octree->addPointsFromInputCloud();

}

void PointPicker::getClickRay(int x, int y,
                              Eigen::Vector3f& p1,
                              Eigen::Vector3f& p2){

    double dX, dY, dZ, dClickY;

    // Convert matrices to doubles
    double mvmatrix[16];
    double projmatrix[16];
    for (int i = 0; i < 16; ++i){
        projmatrix[i] = glarea->camera.projectionMatrix().data()[i];
        mvmatrix[i] = glarea->camera.modelviewMatrix().data()[i];
    }

    // Fetch viewport
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    // Invert y axis
    dClickY = double (glarea->height() - y);

    // Unproject
    gluUnProject ((double) x, dClickY, 0.0, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p1 = Eigen::Vector3f ( (float) dX, (float) dY, (float) dZ );
    gluUnProject ((double) x, dClickY, 1.0f, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p2 = Eigen::Vector3f ( (float) dX, (float) dY, (float) dZ );
}

inline float pointToLineDist(Eigen::Vector3f point, Eigen::Vector3f x1, Eigen::Vector3f x2){
    return (x2-x1).cross(x1-point).squaredNorm()/(x2-x1).squaredNorm();
}

int PointPicker::pick(int x, int y, int source_layer){

    Layer & layer = cm->layerList.layers[source_layer];

    Eigen::Vector3f p1, p2;
    getClickRay(x, y, p1, p2);
    Eigen::Vector3f line = p2-p1;

    int min_index = -1;
    float min_val = FLT_MAX;

    // Need to narrow down candidates

    Eigen::Vector3f& origin = p1;
    Eigen::Vector3f direction = p2-p1;
    std::vector<int> intercept_indices;

    octree->getIntersectedVoxelIndices(origin, direction, intercept_indices);


    // Find point
    for (int i: intercept_indices) {

        // Skip indices not in layer
        if(layer.index[i] == -1)
            continue;

        // Save some memory by using map
        pcl::PointXYZI & p = cm->cloud->points[i];
        Eigen::Vector3f point = Eigen::Vector3f(p.x, p.y, p.z);
        //Eigen::Map<Eigen::Vector3f> point = Eigen::Vector3f::Map(p.data, 3);

        // Skip points not on the ray
        Eigen::Vector3f projPoint = (point.dot(line)/line.dot(line))*line;
        float lineLength = sqrt(line.squaredNorm());
        float distViaPoint = sqrt((p2-projPoint).squaredNorm()) + sqrt((projPoint-p1).squaredNorm());
        if(lineLength-distViaPoint > 0.01)
            continue;

        // Skip points to far from the ray
        /*
        float projDist = (projPoint-point).squaredNorm();
        if(projDist > max_dist)
            continue;
        */

        float dist = pointToLineDist(point, p1, p2);

        if(dist < min_val){
            min_index = i;
            min_val = dist;
        }
    }

    if(min_index == -1)
        qDebug("Cannot find closest point\n");

    return min_index;
}
