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

#include <stdlib.h>
#include <time.h>
#include <vector>
#include <stdexcept>

#include "layer.h"
#include "cloudmodel.h"

inline float rand_range(float from, float to){
    return (rand()/(float)RAND_MAX) * (to-from) + from;
}

Layer::Layer(): gl_index_buffer(QGLBuffer::IndexBuffer)
{
    srand (time(NULL));
    this->active = false;
    this->visible = true;
    this->colour << rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f);

    this->cpu_dirty = true;
    this->gpu_dirty = false;

    bool created = gl_index_buffer.create();

    if(!created){
        qDebug("Did not create buffer");
        throw "a fit";
    }

    gl_index_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    gl_index_buffer.bind();
    gl_index_buffer.allocate(CloudModel::Instance()->cloud->size() * sizeof(int));
    gl_index_buffer.release();

    index.resize(CloudModel::Instance()->cloud->size(), -1);

    // blank index
    for(int & i: index)
        i = -1;

}

bool Layer::sync(){
    if(cpu_dirty & gpu_dirty)
        return false;
    if(cpu_dirty)
        copyToGPU();
    if(gpu_dirty)
        copyFromGPU();
    return true;
}

void Layer::copyToGPU(){
    if(!gl_index_buffer.isCreated()){
        qDebug("GPU buffer not created");
        return;
    }
    gl_index_buffer.bind();
    gl_index_buffer.write(0, &index[0], gl_index_buffer.size());
    gl_index_buffer.release();
    gpu_dirty = false;
    cpu_dirty = false;
}

void Layer::copyFromGPU(){

    if(!gl_index_buffer.isCreated()){
        qDebug("GPU buffer not created");
        return;
    }

    gl_index_buffer.bind();
    if(gl_index_buffer.size() == -1){
        qDebug("GPU buffer not allocated");
        gl_index_buffer.release();
        return;
    }

    if(index.size() == 0)
       index.resize(gl_index_buffer.size());

    gl_index_buffer.read(0, &index[0], gl_index_buffer.size());

    gl_index_buffer.release();
}

void Layer::toggleActive(){
    active = !active;
}

void Layer::toggleVisible(){
    visible = !visible;
}

void Layer::setActive(bool isActive){
    active = isActive;
}
