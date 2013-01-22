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

#include "cloudclean/cloudmodel.h"

#include <QDebug>
#include <QTime>

#include <pcl/filters/filter.h>
#include <pcl/features/integral_image_normal.h>

#include <ctime>
#include <vector>

#include "cloudclean/io.h"

void inline  glError(const char * msg) {
    int err = glGetError();
    if (err) {
        printf("%s : %s\n", msg , gluErrorString(err));
    }
}

// CloudModel* CloudModel::only_instance = NULL;

CloudModel::CloudModel(QObject *parent)
    : QObject(parent), point_buffer(QGLBuffer::VertexBuffer), layerList(this) {
    cloud = pcl::PointCloud<pcl::PointXYZI>
            ::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    x_dim = 0;
    y_dim = 0;
    loaded = false;
}

/*CloudModel * CloudModel::Instance() {
    if (!only_instance)
        only_instance = new CloudModel();
    return only_instance;
}*/

bool CloudModel::saveFile(const char * output_file) {
    // Reconstruct original cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud =
            pcl::PointCloud<pcl::PointXYZI>
            ::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    original_cloud->is_dense = false;
    original_cloud->width = x_dim;
    original_cloud->height = y_dim;
    original_cloud->resize(x_dim*y_dim);
    original_cloud->sensor_origin_[0] = cloud->sensor_origin_[0];
    original_cloud->sensor_origin_[1] = cloud->sensor_origin_[1];
    original_cloud->sensor_origin_[2] = cloud->sensor_origin_[2];
    original_cloud->sensor_origin_[3] = cloud->sensor_origin_[3];
    original_cloud->sensor_orientation_ = cloud->sensor_orientation_;

    pcl::PointXYZI p;
    p.x = p.y = p.z = p.intensity = NAN;

    /// Init with nans
    for (unsigned int i = 0; i < cloud_to_grid_map.size(); i++) {
        original_cloud->points[i] = p;
    }

    // Copy in all visible layers
    std::vector<Layer> & layers = layerList.layers;
    for (unsigned int l = 0; l < layers.size(); l++) {
        layers[l].sync();
        for (unsigned int i = 0; i < layers[l].index.size(); i++) {
            int idx = layers[l].index[i];
            // for some reason bad indices sneak in here
            // that are not -1
            if (idx == -1)
                continue;
            original_cloud->points[cloud_to_grid_map[idx]] = cloud->points[idx];
        }
    }


    save_ptx(output_file, original_cloud);
    return true;
}

bool CloudModel::createBuffers() {
    if (!loaded)
        return false;

    point_buffer.create();
    point_buffer.setUsagePattern(QGLBuffer::DynamicDraw);
    if ( !point_buffer.bind() ) {
        qWarning() << "Could not bind vertex buffer to the context";
        return false;
    }
    point_buffer.allocate(cloud->points.size() * sizeof(float) * 4);

    float data[4];
    for (int i = 0; i < static_cast<int>(cloud->size()); i++) {
        data[0] = cloud->points[i].x;
        data[1] = cloud->points[i].y;
        data[2] = cloud->points[i].z;
        data[3] = cloud->points[i].intensity;
        int offset = 4*sizeof(float)*i;
        point_buffer.write(offset, reinterpret_cast<const void *> (data),
                           sizeof(data));
    }

    point_buffer.release();

    /// Create first layer
    layerList.newLayer();

    Layer & layer = layerList.layers[0];

    // Change initial layers colour
    layer.colour = Eigen::Vector3f(1.0f, 1.0f, 1.0f);

    /// Initialise the first layer to include all points
    for (unsigned int i = 0; i < cloud->size(); i++) {
        layer.index[i] = i;
    }

    layer.cpu_dirty = true;
    layer.sync();

    qDebug("Buffers created & loaded.");
    return true;
}

void normal_estimation(pcl::PointCloud<pcl::PointXYZI>
                       ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

void filter(pcl::PointCloud<pcl::PointXYZI>& cloud,
            pcl::PointCloud<pcl::Normal>& normals, std::vector<int> &index) {
    index.resize(normals.points.size());
    size_t j = 0;
    for (size_t i = 0; i < normals.points.size(); ++i) {
           if (!pcl_isfinite(normals.points[i].normal_x) ||
               !pcl_isfinite(normals.points[i].normal_y) ||
               !pcl_isfinite(normals.points[i].normal_z))
             continue;
           normals.points[j] = normals.points[i];
           index[j] = i;
           j++;
     }

    // Resize to the correct size
    normals.points.resize(j);
    index.resize(j);
    normals.height = 1;
    normals.width  = j;
    normals.is_dense = true;

    // Filter cloud
    for (unsigned int i = 0; i < j; i++) {
        cloud.points[i] = cloud.points[index[i]];
    }
    cloud.points.resize(j);
    cloud.height = 1;
    cloud.width  = j;
    cloud.is_dense = true;
}

bool CloudModel::loadFile(const char * input_file, int subsample) {
    qDebug("Subsample: %d\n", subsample);

    QTime t;
    t.start();

    cloud = read_ptx(input_file, subsample);

    qDebug("File loaded in %d ms with %lu points", t.elapsed(), cloud->size());

    x_dim = cloud->width;
    y_dim = cloud->height;

    t.start();
    normals = pcl::PointCloud<pcl::Normal>
            ::Ptr(new pcl::PointCloud<pcl::Normal>);
    normal_estimation(cloud, normals);
    qDebug("Normals calculated in %d ms", t.elapsed());

    t.start();
    filter(*cloud, *normals, cloud_to_grid_map);
    qDebug("Normals filtered in %d ms", t.elapsed());

    if (loaded)
        layerList.reset();

    loaded = true;

    t.start();
    createBuffers();
    qDebug("Points loaded to GPU in %d ms", t.elapsed());

    return true;
}

bool CloudModel::isLoaded() {
    return loaded;
}

class PointIdx {
 public:
    int x;
    int y;
    PointIdx(int x, int y): x(x), y(y) {}
    PointIdx(): x(0), y(0) {}
};

class PointIdxPair {
 public:
    PointIdx p1;
    PointIdx p2;
    PointIdxPair(PointIdx p1, PointIdx p2): p1(p1), p2(p2) {}
    PointIdxPair(int p1x, int p1y, int p2x, int p2y)
        :p1(PointIdx(p1x, p1y)), p2(PointIdx(p2x, p2y)) {}
};

bool isValidPoint(PointIdx & point, int width, int height,
                  int index, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    // calculate current x and y
    int cur_x = index%width;
    int cur_y = index/width;

    // Bounds check
    int x_pos = cur_x + point.x;
    int y_pos = cur_y + point.y;
    if (x_pos > width-1 || x_pos < 0)
        return false;
    if (y_pos > height-1 || y_pos < 0)
        return false;

    // Index with NaN value is not valid
    int idx = y_pos*width + x_pos;
    if (cloud->points[idx].x != cloud->points[idx].x)
        return false;

    return true;
}

void normal_estimation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr normals) {
    int length = cloud->points.size();
    int width = cloud->width;
    int height = cloud->height;
    normals->points.resize(length);

    // Relative point anticlockwise around (0,0)
    PointIdx pointList[8] = {
        PointIdx(1, 0),
        PointIdx(1, -1),
        PointIdx(0, -1),
        PointIdx(-1, -1),
        PointIdx(-1, 0),
        PointIdx(-1, 1),
        PointIdx(0, 1),
        PointIdx(1, 1)
    };

    for (int i = 0; i < length; ++i) {
        // Don't calculate normal for NANs
        if (cloud->points[i].x != cloud->points[i].x) {
            normals->points[i].data_n[0] = NAN;
            normals->points[i].data_n[1] = NAN;
            normals->points[i].data_n[2] = NAN;
            normals->points[i].data_n[3] = NAN;
            continue;
        }

        // Normal calculation vars
        Eigen::Vector3f agregate_n(0.0f, 0.0f, 0.0f);
        int face_count = 0;

        // Calculate normal from 8 triangles
        int next = 0;
        int first = -1;
        while (true) {
            // Find valid indices around 0,0
            int p1idx = -1, p2idx = -1;

            while (next < 8) {
                if (isValidPoint(pointList[next], width, height, i, cloud)) {
                    p1idx = next++;
                    break;
                }
                next++;
            }

            while (next < 9) {
                // Don't want two of the same indices in different orders
                if (next == 8 && face_count == 1)
                    break;

                // Loop around
                if (next == 8 && face_count > 1) {
                    p2idx = first;
                    break;
                }

                int idx = next % 8;
                if (isValidPoint(pointList[idx], width, height, i, cloud)) {
                    p2idx = idx;
                    break;
                }
                next++;
            }

            // Stop if no points in range were found
            if (p1idx == -1 || p2idx == -1)
                break;

            if (first == -1)
                first = p1idx;

            PointIdxPair relativePair(pointList[p1idx], pointList[p2idx]);

            // calculate current x and y
            int cur_x = i%width;
            int cur_y = i/width;

            // Calculate absolute indices
            int idx1 = (cur_y+relativePair.p1.y)*width
                    + (cur_x+relativePair.p1.x);
            int idx2 = (cur_y+relativePair.p2.y)*width
                    + (cur_x+relativePair.p2.x);

            pcl::PointXYZI & cp0 = cloud->points[i];
            pcl::PointXYZI & cp1 = cloud->points[idx1];
            pcl::PointXYZI & cp2 = cloud->points[idx2];

            Eigen::Vector3f p0(cp0.x, cp0.y, cp0.z);
            Eigen::Vector3f p1(cp1.x, cp1.y, cp1.z);
            Eigen::Vector3f p2(cp2.x, cp2.y, cp2.z);

            // Cross product to get normal
            Eigen::Vector3f vec1 = p1 - p0;
            Eigen::Vector3f vec2 = p2 - p0;
            Eigen::Vector3f tmp = vec1.cross(vec2).normalized();

            // Normalized 0 0 0 == NaN
            if (tmp.x() != tmp.x() ||
               tmp.y() != tmp.y() ||
               tmp.z() != tmp.z()
            )
                continue;

            agregate_n += tmp;

            face_count++;
        }

        // Face count can be 0 or normals can cancel
        // Should set to point to camera
        if (face_count == 0) {
            normals->points[i].data_n[0] = NAN;
            normals->points[i].data_n[1] = NAN;
            normals->points[i].data_n[2] = NAN;
            normals->points[i].data_n[3] = NAN;
            continue;
        }

        Eigen::Vector3f normal = agregate_n.normalized();

        pcl::flipNormalTowardsViewpoint(cloud->points[i], 0, 0, 0, normal);

        normals->points[i].data_n[0] = normal(0);
        normals->points[i].data_n[1] = normal(1);
        normals->points[i].data_n[2] = normal(2);
        normals->points[i].data_n[3] = 1.0f;
    }
}
