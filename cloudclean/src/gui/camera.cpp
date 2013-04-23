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

#include "gui/camera.h"
#include <cmath>
#include <mutex>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <QDebug>
#include <iostream>

using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::AngleAxis;

Camera::Camera() {
    //mtx_ = new std::mutex();
    fov_ = 60.0f;
    aspect_ = 1.0f;
    depth_near_ = 1.0f;
    depth_far_ = 10000.0f;

    rotation_ = AngleAxis<float>(-M_PI/2, Vector3f(1, 0, 0));

    translation_ = Vector3f(0, 0, 0);

    projection_dirty_ = true;
}

Camera::~Camera() {
    //delete mtx_;
}

void Camera::setFoV(float fov) {
    fov_ = fov;
    projection_dirty_ = true;
}

void Camera::setAspect(float aspect) {
    aspect_ = aspect;
    projection_dirty_ = true;
}

void Camera::setDepthRange(float near, float far) {
    depth_near_ = near;
    depth_far_ = far;
    projection_dirty_ = true;
}

void Camera::setPosition(const Eigen::Vector3f& pos) {
    translation_ = pos;
}


void Camera::recalculateProjectionMatrix() {
    // Code from Mesa project, src/glu/sgi/libutil/project.c
    projection_matrix_.setIdentity();
    float radians = fov_ / 2 * M_PI / 180;

    float deltaZ = depth_far_ - depth_near_;
    float sine = sin(radians);
    if ((deltaZ == 0) || (sine == 0) || (aspect_ == 0)) {
        return;
    }
    float cotangent = cos(radians) / sine;

    projection_matrix_(0, 0) = cotangent / aspect_;
    projection_matrix_(1, 1) = cotangent;
    projection_matrix_(2, 2) = -(depth_far_ + depth_near_) / deltaZ;
    projection_matrix_(3, 2) = -1;
    projection_matrix_(2, 3) = -2 * depth_near_ * depth_far_ / deltaZ;
    projection_matrix_(3, 3) = 0;
    projection_dirty_ = false;
}

void Camera::setProjectionMatrix(const Eigen::Affine3f& projection) {
    projection_matrix_ = projection;
    projection_dirty_ = false;
}

Eigen::Affine3f Camera::modelviewMatrix() {
    return rotation_  * Eigen::Translation3f(translation_) * Eigen::Affine3f::Identity();
}

Eigen::Affine3f Camera::projectionMatrix() const {
    if (projection_dirty_) {
        const_cast<Camera*>(this)->recalculateProjectionMatrix();
    }
    return projection_matrix_;
}

void Camera::translate(const Eigen::Vector3f& pos) {
    translation_ = Eigen::Translation3f(rotation_.inverse() * pos) * translation_;
}

void Camera::rotate2D(float x, float y) {
    Vector2f rot = Vector2f(x, y);

    AngleAxis<float> rotX(rot.x(), Vector3f::UnitY()); // look left right
    AngleAxis<float> rotY(rot.y(), Vector3f::UnitX()); // look up down

    rotation_ = (rotX * rotY) * rotation_;

    auto clamp = [] (double num, double low, double high) {
        if (num > high)
            return high;
        if (num < low)
            return low;
        return num;
    };

    Eigen::Matrix3f r = rotation_.toRotationMatrix();
    double roll = -atan2(r(0,2), r(1, 2));
    double pitch = acos(r(2,2));
    //double yaw = atan2(r(2, 0), r(2, 1));


    Vector3f dir = rotation_ * Vector3f::UnitZ();
    double dotp = dir.dot(Vector3f::UnitZ());

    double sign = dir.dot(Vector3f::UnitY()) > 0 ? 1.0 : -1.0;
    double angle = sign * acos(dotp);

/*
    qDebug() << "Y angle" << angle;
    qDebug() << "Y angle (DEG)" << (angle/M_PI) * 180;
    qDebug() << "Roll" << roll;
    qDebug() << "Pitch" << pitch;
    qDebug() << "Yaw" << yaw;
*/
    double correction_factor = 1.0 - fabs(pitch-M_PI/2)/(M_PI/2);
    correction_factor = -0.5 + 1.5 * correction_factor;
    correction_factor = clamp(correction_factor, 0, 1);

    if(angle < 0)
        correction_factor = 0;

    //qDebug() << "Correction factor:" << correction_factor;

    AngleAxis<float> roll_correction(correction_factor*-roll, Vector3f::UnitZ());
    rotation_ = roll_correction * rotation_;
    rotation_.normalize();
}

void Camera::adjustFov(int val) {
    // Mouse seems to move in increments of 120
    val = -val/60.0f;
    if (fov_ + val < 170.0f && fov_ + val > 2.0f)
        setFoV(fov_ + val);
}
