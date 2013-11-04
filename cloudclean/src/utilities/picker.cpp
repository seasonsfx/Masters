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


Picker::Picker(GLWidget *glwidget, CloudList * cl, std::function<void (int)> callback) {
    callback_ = callback;
    glwidget_ = glwidget;
    cl_ = cl;
}


bool Picker::eventFilter(QObject *object, QEvent *event){
    if(QApplication::keyboardModifiers() == Qt::SHIFT)
        return false;

    switch(event->type()){
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    default:
        return false;
    }
}

bool Picker::mouseReleaseEvent(QMouseEvent * event){

    int idx = pick(event->x(), event->y(), glwidget_->width(),
                   glwidget_->height(), 0.02,
                   glwidget_->camera_.projectionMatrix(),
                   glwidget_->camera_.modelviewMatrix(),
                   cl_->active_);

    if(idx != -1)
        callback_(idx);

    return true;
}
