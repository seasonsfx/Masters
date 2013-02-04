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
#include <math.h>
#include <Eigen/LU>
#include <QDebug>

using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::AngleAxis;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Camera::Camera() {
    mFoV = 60.0f;
    mAspect = 1.0f;
    mDepthNear = 1.0f;
    mDepthFar = 100.0f;
    mPosition = Vector3f(0, 0, 0);
    mLookAt = Vector3f(0, 0, 1);

    startSideAxis =     Vector3f(1, 0, 0);
    startUpAxis =       Vector3f(0, 0, 1);
    startForwardAxis =  Vector3f(0, 1, 0);

    forward = startForwardAxis;
    mUp =     startUpAxis;

    mModelviewMatrixDirty = true;
    mProjectionMatrixDirty = true;
    mMouseDown = false;
    moveSensitivity = 0.05;
    mObjectOrientationMatrix.setIdentity();
    mouseButtonPressed = 0;
}

Camera::~Camera() {
}

void Camera::setFoV(float fov) {
    mFoV = fov;
    mProjectionMatrixDirty = true;
}

void Camera::setAspect(float aspect) {
    mAspect = aspect;
    mProjectionMatrixDirty = true;
}

void Camera::setDepthRange(float near, float far) {
    mDepthNear = near;
    mDepthFar = far;
    mProjectionMatrixDirty = true;
}

void Camera::setPosition(const Eigen::Vector3f& pos) {
    mPosition = pos;
    mModelviewMatrixDirty = true;
}

void Camera::adjustPosition(const Eigen::Vector3f& pos) {
    Vector3f side = forward.cross(mUp).normalized();
    Vector3f up = side.cross(forward);
    Vector3f offset  = pos.x() * side + pos.y() * up + pos.z() * forward;
    mPosition += offset;
    mModelviewMatrixDirty = true;
}

void Camera::setLookAt(const Eigen::Vector3f& lookat) {
    mLookAt = lookat;
    forward = (mLookAt - mPosition).normalized();
    mModelviewMatrixDirty = true;
}

void Camera::setUp(const Eigen::Vector3f& up) {
    mUp = up;
    mModelviewMatrixDirty = true;
}

void Camera::setDirection(const Eigen::Vector3f& dir) {
    setLookAt(mPosition + dir);
}

void Camera::recalculateModelviewMatrix() {
    // Code from Mesa project, src/glu/sgi/libutil/project.c
    mModelviewMatrixDirty = false;
    Vector3f side = forward.cross(mUp).normalized();
    // Recompute up vector, using cross product
    Vector3f up = side.cross(forward);

    mModelviewMatrix.setIdentity();
    mModelviewMatrix.linear() << side.transpose(), up.transpose(),
            -forward.transpose();
    mModelviewMatrix.translate(-mPosition);
    mModelviewMatrix *=mObjectOrientationMatrix.matrix();\
}

void Camera::recalculateProjectionMatrix() {
    // Code from Mesa project, src/glu/sgi/libutil/project.c
    mProjectionMatrixDirty = false;
    mProjectionMatrix.setIdentity();
    float radians = mFoV / 2 * M_PI / 180;

    float deltaZ = mDepthFar - mDepthNear;
    float sine = sin(radians);
    if ((deltaZ == 0) || (sine == 0) || (mAspect == 0)) {
        return;
    }
    float cotangent = cos(radians) / sine;

    mProjectionMatrix(0, 0) = cotangent / mAspect;
    mProjectionMatrix(1, 1) = cotangent;
    mProjectionMatrix(2, 2) = -(mDepthFar + mDepthNear) / deltaZ;
    mProjectionMatrix(3, 2) = -1;
    mProjectionMatrix(2, 3) = -2 * mDepthNear * mDepthFar / deltaZ;
    mProjectionMatrix(3, 3) = 0;
}

void Camera::setObjectOrientationMatrix(const Eigen::Affine3f& objectorient) {
    mObjectOrientationMatrix = objectorient;
    mModelviewMatrixDirty = true;
}

void Camera::setModelviewMatrix(const Eigen::Affine3f& modelview) {
    mModelviewMatrix = modelview;
    mModelviewMatrixDirty = false;
}

void Camera::setProjectionMatrix(const Eigen::Affine3f& projection) {
    mProjectionMatrix = projection;
    mProjectionMatrixDirty = false;
}

Eigen::Affine3f Camera::modelviewMatrix() const {
    if (mModelviewMatrixDirty) {
        const_cast<Camera*>(this)->recalculateModelviewMatrix();
    }
    return mModelviewMatrix;
}

Eigen::Affine3f Camera::projectionMatrix() const {
    if (mProjectionMatrixDirty) {
        const_cast<Camera*>(this)->recalculateProjectionMatrix();
    }
    return mProjectionMatrix;
}

void Camera::mouseDown(int x, int y, int button) {
    mouseButtonPressed = button;
    mouseStart << x*moveSensitivity, y*moveSensitivity;
    savedForward = forward;
    savedObjectOrientationMatrix = mObjectOrientationMatrix;
    mMouseDown = true;
}
void Camera::mouseRelease(int x, int y) {
    mouseButtonPressed = 0;
    mMouseDown = false;
}

// TODO(Rickert): object roation should be more inteligent
// Read: http://nehe.gamedev.net/tutorial/loading_and_moving_through_a_3d_world/22003/
void Camera::mouseMove(int x, int y) {
    if (!mMouseDown)
        return;
    Vector2f rot = Vector2f(x*moveSensitivity, y*moveSensitivity) - mouseStart;

    if (mouseButtonPressed == LEFT_BTN) {
        Vector3f side = forward.cross(startUpAxis).normalized();

        AngleAxis<float> rotX(-rot.x()*moveSensitivity, startUpAxis);
        AngleAxis<float> rotY(-rot.y()*moveSensitivity, side);

        forward = (rotX * rotY) * savedForward;
        forward.normalize();

        Vector3f side_after = forward.cross(startUpAxis).normalized();

        float dot = side.dot(side_after);

        // Still not working 100%
        if (dot < 0.9) {
            forward = savedForward;
        }
    } else if (mouseButtonPressed == RIGHT_BTN) {
        Vector3f side = forward.cross(mUp).normalized();
        Vector3f up = side.cross(forward);

        AngleAxis<float> rotX(-rot.x()*moveSensitivity, up);
        AngleAxis<float> rotY(-rot.y()*moveSensitivity, side);
        mObjectOrientationMatrix = rotX * rotY * savedObjectOrientationMatrix;
    }

    mModelviewMatrixDirty = true;
}

void Camera::mouseWheel(int val) {
    // Mouse seems to move in 120 increments
    val = -val/60.0f;
    if (mFoV + val < 170.0f && mFoV + val > 2.0f)
        setFoV(mFoV + val);
}
