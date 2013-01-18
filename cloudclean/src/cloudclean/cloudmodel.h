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

#ifndef APPDATA_H
#define APPDATA_H

#include "cloudclean_global.h"

#include <QObject>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>

#include "layerlist.h" // includes gl stuff
#include <GL/glu.h>

// TODO: a cloud model should probably own a GL context?
class DLLSPEC CloudModel : public QObject
{
    Q_OBJECT
public:
    std::vector< int >                              cloud_to_grid_map;
    int                                             x_dim;
    int                                             y_dim;
    bool                                            loaded;

    // Where is the ground plane? Could be useful to detect trees

    pcl::PointCloud<pcl::PointXYZI>::Ptr            cloud;
    QGLBuffer                                       point_buffer;
    LayerList                                       layerList;
    pcl::PointCloud<pcl::Normal>::Ptr               normals;

public:
    static CloudModel* Instance();
    bool createBuffers();
    bool loadFile(const char * input_file, int subsample);
    bool saveFile(const char * output_file);
    bool isLoaded();

private:
    static CloudModel* only_instance;
    explicit CloudModel(QObject *parent = 0);
    CloudModel(CloudModel const&);
    CloudModel& operator=(CloudModel const&) { return *this; }

signals:

public slots:

};

#endif // APPDATA_H
