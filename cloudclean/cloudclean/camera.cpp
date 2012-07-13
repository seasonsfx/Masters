#include "camera.h"
#include <math.h>
#include <Eigen/LU>
#include <QDebug>

using namespace Eigen;

Camera::Camera()
{
    mFoV = 45.0f;
    mAspect = 1.0f;
    mDepthNear = 1.0f;
    mDepthFar = 100.0f;
    mPosition = Vector3f(0, 0, -20);
    mLookAt = Vector3f(0, 0, 1);
    mUp = Vector3f(0, 1, 0);
    mModelviewMatrixDirty = true;
    mProjectionMatrixDirty = true;
    mMouseDown = false;
    moveSensitivity = 0.05;
    mObjectOrientationMatrix.setIdentity();
}

Camera::~Camera()
{
}

void Camera::setFoV(float fov)
{
    mFoV = fov;
    mProjectionMatrixDirty = true;
}

void Camera::setAspect(float aspect)
{
    mAspect = aspect;
    mProjectionMatrixDirty = true;
}

void Camera::setDepthRange(float near, float far)
{
    mDepthNear = near;
    mDepthFar = far;
    mProjectionMatrixDirty = true;
}

void Camera::setPosition(const Eigen::Vector3f& pos)
{
    mPosition = pos;
    mModelviewMatrixDirty = true;
}

void Camera::adjustPosition(const Eigen::Vector3f& pos)
{
    // Our looking direction
    Vector3f forward = (mLookAt - mPosition).normalized();
    Vector3f side = forward.cross(mUp).normalized();
    Vector3f up = side.cross(forward);

    Vector3f offset  = pos.x() * side + pos.y() * up + pos.z() * forward;

    mPosition += offset;
    mLookAt += offset;
    mModelviewMatrixDirty = true;
}

void Camera::setLookAt(const Eigen::Vector3f& lookat)
{
    mLookAt = lookat;
    mModelviewMatrixDirty = true;
}

void Camera::setUp(const Eigen::Vector3f& up)
{
    mUp = up;
    mModelviewMatrixDirty = true;
}

void Camera::setDirection(const Eigen::Vector3f& dir)
{
    setLookAt(mPosition + dir);
}

void Camera::recalculateModelviewMatrix()
{
    // Code from Mesa project, src/glu/sgi/libutil/project.c
    mModelviewMatrixDirty = false;
    // Our looking direction
    Vector3f forward = (mLookAt - mPosition).normalized();

    Vector3f side = forward.cross(mUp).normalized();

    // Recompute up vector, using cross product
    Vector3f up = side.cross(forward);

    mModelviewMatrix.setIdentity();
    mModelviewMatrix.linear() << side.transpose(), up.transpose(), -forward.transpose();
    mModelviewMatrix.translate(-mPosition);
    mModelviewMatrix *=mObjectOrientationMatrix.matrix();
    qDebug("mPosition (%f, %f, %f)", mPosition.x(), mPosition.y(), mPosition.z());
}

void Camera::recalculateProjectionMatrix()
{
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

void Camera::setObjectOrientationMatrix(const Eigen::Affine3f& objectorient){
    mObjectOrientationMatrix = objectorient;
    mModelviewMatrixDirty = true;
}

void Camera::setModelviewMatrix(const Eigen::Affine3f& modelview)
{
    mModelviewMatrix = modelview;
    mModelviewMatrixDirty = false;
}

void Camera::setProjectionMatrix(const Eigen::Affine3f& projection)
{
    mProjectionMatrix = projection;
    mProjectionMatrixDirty = false;
}

Eigen::Affine3f Camera::modelviewMatrix() const
{
    if (mModelviewMatrixDirty) {
        const_cast<Camera*>(this)->recalculateModelviewMatrix();
    }
    return mModelviewMatrix;
}

Eigen::Affine3f Camera::projectionMatrix() const
{
    if (mProjectionMatrixDirty) {
        const_cast<Camera*>(this)->recalculateProjectionMatrix();
    }
    return mProjectionMatrix;
}

void Camera::mouseDown(int x, int y){
    mouseStart << x*moveSensitivity, y*moveSensitivity;
    qDebug("Mouse start: (%f, %f)", mouseStart.x(), mouseStart.y());
    savedLookAt = mLookAt;
    mMouseDown = true;
}
void Camera::mouseRelease(int x, int y){
    mMouseDown = false;
}
void Camera::mouseMove(int x, int y){
    if(!mMouseDown)
        return;
    Vector2f rot = Vector2f(x*moveSensitivity,y*moveSensitivity) - mouseStart;

    Vector3f forward = (mLookAt - mPosition).normalized();
    Vector3f side = forward.cross(mUp).normalized();
    Vector3f up = side.cross(forward);


    up = Vector3f(0,1,0); // Keep left right look level

    AngleAxis<float> rotX(-rot.x()*moveSensitivity, up);
    AngleAxis<float> rotY(-rot.y()*moveSensitivity, side);

    mLookAt = (rotX * rotY * (savedLookAt-mPosition)) + mPosition;

    //qDebug("Offset: (%f, %f)", offset.x(), offset.y());
    qDebug("Look at: (%f, %f, %f)", mLookAt.x(), mLookAt.y(), mLookAt.z());

    mModelviewMatrixDirty = true;
}

void Camera::mouseWheel(int val){
    // Mouse seems to move in 120 increments
    qDebug("Wheel: %d", val);
    val = val/120.0f;
    adjustPosition(0,0, val*0.2);
}
