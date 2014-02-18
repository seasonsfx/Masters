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

#include "utilities/picker.h"

#include <QApplication>
#include <QEvent>
#include <QMouseEvent>

#include "utilities/pointpicker.h"
#include "gui/glwidget.h"
#include "model/cloudlist.h"


Picker::Picker(GLWidget *glwidget, CloudList * cl, std::function<void (int)> callback, bool * enabled) {
    callback_ = callback;
    glwidget_ = glwidget;
    cl_ = cl;
    enabled_ = enabled;

//    QFile file(":/picking.frag");
//    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
//        qDebug() << "---------= cant read frag -----------------";
//    } else {
//        while (!file.atEnd()) {
//            QString line = file.readLine();
//            qDebug() << line;
//        }
//    }


    //
    // Load shader program
    //
    bool succ = program_3d_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/picking.vert"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_3d_.log();
    succ = program_3d_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/picking.frag"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_3d_.log();
    succ = program_3d_.link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_3d_.log();
        qWarning() << "Exiting...";
        abort();
    }


    //
    // Resolve uniforms
    //
    program_3d_.bind(); CE();
    uni_sampler_ = program_3d_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_projection_ = program_3d_.uniformLocation("projection"); RC(uni_projection_);
    uni_modelview_ = program_3d_.uniformLocation("modelview"); RC(uni_modelview_);
    program_3d_.release(); CE();

    //
    // Set up textures & vao
    //
    glGenTextures(1, &texture_id_); CE();
    glGenTextures(1, &picking_texture_id_); CE();
    glGenTextures(1, &depth_texture_id_); CE();
    glGenVertexArrays(1, &vao_);
    glGenFramebuffers(1, &fbo_);
}

Picker::~Picker(){
    glDeleteTextures(1, &texture_id_); CE();
    glDeleteTextures(1, &picking_texture_id_); CE();
    glDeleteTextures(1, &depth_texture_id_); CE();
    glDeleteVertexArrays(1, &vao_);CE();
    glDeleteFramebuffers(1, &fbo_);CE();
}

uint Picker::renderPick(int x, int y){

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);CE();
    // Create the texture object for the primitive information buffer
    glBindTexture(GL_TEXTURE_2D, picking_texture_id_);CE();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32UI, glwidget_->width(), glwidget_->height(),
                    0, GL_RGB_INTEGER, GL_UNSIGNED_INT, NULL);CE();
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                    picking_texture_id_, 0);CE();

    // Create the texture object for the depth buffer
    glBindTexture(GL_TEXTURE_2D, depth_texture_id_);CE();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, glwidget_->width(), glwidget_->height(),
                    0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);CE();
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                    depth_texture_id_, 0);CE();

    // Disable reading to avoid problems with older GPUs
    glReadBuffer(GL_NONE);CE();

    // Verify that the FBO is correct
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);CE();

    if (status != GL_FRAMEBUFFER_COMPLETE) {
        printf("FB error, status: 0x%x\n", status);
        return false;
    }


    glClearColor(1.0, 1.0, 1.0, 1.0);CE();
    glEnable(GL_DEPTH_TEST);CE();
    glEnable(GL_MULTISAMPLE);CE();
    glEnable(GL_POINT_SMOOTH);CE();
    glPointSize(glwidget_->pointRenderSize());CE();
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); CE();


    program_3d_.bind(); CE();


    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    auto buff = glwidget_->gld()->color_lookup_buffer_;CE();
    auto id = buff->bufferId();CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, id); CE();

    glBindVertexArray(vao_);CE();

    boost::shared_ptr<PointCloud> pc = cl_->active_;
    boost::shared_ptr<CloudGLData> cd = glwidget_->gld()->cloudgldata_.at(pc);

    /////////

    // Point buffer
    cd->point_buffer_->bind(); CE();
    glEnableVertexAttribArray(0); CE();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
    glEnableVertexAttribArray(1); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                          reinterpret_cast<const void *>(offset)); CE();
    cd->point_buffer_->release(); CE();

    // Label buffer
    cd->label_buffer_->bind(); CE();
    glEnableVertexAttribArray(2); CE(); CE();
    glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0); CE();
    cd->label_buffer_->release(); CE();

    // Flag buffer
    cd->flag_buffer_->bind(); CE();
    glEnableVertexAttribArray(3); CE();
    glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
    cd->flag_buffer_->release(); CE();

    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       glwidget_->camera_.projectionMatrix().data());CE();

    glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                       (glwidget_->camera_.modelviewMatrix()*pc->modelview())
                       .data());CE();

    cd->draw(vao_);

    /////////
    glBindVertexArray(0);CE();
    program_3d_.release(); CE();

    // read point
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glReadBuffer(GL_COLOR_ATTACHMENT0);CE();
    uint data[3];
    glReadPixels(x, glwidget_->height() - y, 1, 1, GL_RGB_INTEGER, GL_UNSIGNED_INT, &data);CE();
    glReadBuffer(GL_NONE);CE();
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);CE();

    // Restore the default framebuffer
    glBindTexture(GL_TEXTURE_2D, 0);CE();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);CE();

    qDebug() << "render pick" << data[0];

    if(data[0] == 1065353216)
        return -1;

    return data[0];
}

bool Picker::eventFilter(QObject *object, QEvent *event){
    if(QApplication::keyboardModifiers() == Qt::SHIFT || !*enabled_)
        return false;

    switch(event->type()){
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    default:
        return false;
    }
}

bool Picker::mouseReleaseEvent(QMouseEvent * event){

//    int idx = pick(event->x(), event->y(), glwidget_->width(),
//                   glwidget_->height(), 0.02,
//                   glwidget_->camera_.projectionMatrix(),
//                   glwidget_->camera_.modelviewMatrix(),
//                   cl_->active_);

    uint idx = renderPick(event->x(), event->y());

    qDebug() << idx << "picked";

    if(idx != -1 && callback_!=nullptr)
        callback_(idx);

    return true;
}
