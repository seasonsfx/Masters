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

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_CAMERA_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_CAMERA_H_

#include <Eigen/Geometry>
#include "appexport.h"

// Contains code from kgllib
namespace std {
    class mutex;
}

class DLLSPEC Camera {
 public:
    Camera();

    ~Camera();

    void setViewport(int x, int y, int width, int height);

    void setFoV(float fov);

    void setAspect(float aspect);

    void setDepthRange(float near, float far);

    void setPosition(const Eigen::Vector3f& pos);
    void setPosition(float x, float y, float z)  {
        setPosition(Eigen::Vector3f(x, y, z));
    }

    void adjustPosition(const Eigen::Vector3f& pos);
    void adjustPosition(float x, float y, float z)  {
        adjustPosition(Eigen::Vector3f(x, y, z));
    }

    void setLookAt(const Eigen::Vector3f& lookat);
    void setLookAt(float x, float y, float z)  {
        setLookAt(Eigen::Vector3f(x, y, z));
    }

    void setUp(const Eigen::Vector3f& up);
    void setUp(float x, float y, float z)  { setUp(Eigen::Vector3f(x, y, z)); }

    void setDirection(const Eigen::Vector3f& dir);
    void setDirection(float x, float y, float z)  {
        setDirection(Eigen::Vector3f(x, y, z));
    }

    Eigen::Vector3f position() const  { return mPosition; }
    Eigen::Vector3f lookAt() const  { return mLookAt; }
    Eigen::Vector3f up() const  { return mUp; }

    void setObjectOrientationMatrix(const Eigen::Affine3f& objectorient);
    void setModelviewMatrix(const Eigen::Affine3f& modelview);
    void setProjectionMatrix(const Eigen::Affine3f& projection);

    Eigen::Affine3f modelviewMatrix() const;

    Eigen::Affine3f projectionMatrix() const;

    void mouseDown(int x, int y, int button);
    void mouseRelease(int x, int y);
    void mouseMove(int x, int y);
    void mouseWheel(int val, float x = 0, float y = 0);

 private:
    void recalculateModelviewMatrix();
    void recalculateProjectionMatrix();

 public:
    static const int LEFT_BTN = 1;
    static const int RIGHT_BTN = 2;

 private:
    int mouseButtonPressed;
    Eigen::Vector2f mouseStart;
    bool mMouseDown;
    float moveSensitivity;

    Eigen::Vector3f startSideAxis;
    Eigen::Vector3f startUpAxis;
    Eigen::Vector3f startForwardAxis;

    Eigen::Vector3f forward;
    Eigen::Vector3f savedForward;


    Eigen::Affine3f savedObjectOrientationMatrix;


    Eigen::Vector3f mPosition;
    Eigen::Vector3f mLookAt;
    Eigen::Vector3f mUp;
    float mFoV, mAspect, mDepthNear, mDepthFar;

    Eigen::Affine3f mObjectOrientationMatrix;
    bool mObjectOrientationMatrixDirty;
    Eigen::Affine3f mModelviewMatrix;
    bool mModelviewMatrixDirty;
    Eigen::Affine3f mProjectionMatrix;
    bool mProjectionMatrixDirty;

    std::mutex * mutex_;
};

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_CAMERA_H_
