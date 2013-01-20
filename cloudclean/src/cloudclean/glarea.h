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

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_GLAREA_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_GLAREA_H_

#include <ctime>
#include <QtGlobal>

#include "gl_global.h"

#include "cloudclean/cloudclean_global.h"

#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include <QMouseEvent>
#include <QMutex>

#include <GL/glu.h>

#include "cloudclean/cloudmodel.h"
#include "cloudclean/camera.h"
#include "cloudclean/pointpicker.h"

#ifdef Q_WS_X11
    #include <GL/glx.h>
    #undef KeyPress  // Defined in X11/X.h, interferes with QEvent::KeyPress
#endif

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include <CL/cl_gl.h>

class EditPluginInterface;
class PluginManager;

class DLLSPEC GLArea : public QGLWidget {
    Q_OBJECT

 public:
    GLArea(QWidget* parent = 0, PluginManager *pm = NULL,
            CloudModel *cm = NULL);

    bool prepareShaderProgram(QGLShaderProgram& shader,
                               const QString& vertexShaderPath,
                               const QString& fragmentShaderPath,
                               const QString &geometryShaderPath);

    Eigen::Vector2f normalized_mouse(int x, int y);

    void modelReloaded();

 protected:
    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();
    void updateFps(float frameTime);

    void mouseDoubleClickEvent(QMouseEvent * event);
    void mouseMoveEvent(QMouseEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);
    void wheelEvent(QWheelEvent * event);
    void keyPressEvent(QKeyEvent * event);
    bool eventFilter(QObject *object, QEvent *event);

 public slots:

 private:
    QFont qFont;

    QGLFormat                               glFormat;

    int                                     start_move_x;
    int                                     start_move_y;

    float                                   cfps;
    float                                   lastTime;

    int                                     point_size;

 public:
    QGLShaderProgram                        point_shader;

    cl_platform_id                          platform;
    cl_device_id                            device;
    cl_context                              clcontext;
    cl_command_queue                        cmd_queue;

    Qt::MouseButton                         mouseDown;
    bool                                    moved;

    Camera                                  camera;

    CloudModel *                            cm;
    PluginManager *                         pm;
    PointPicker *                           pp;
};

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_GLAREA_H_
