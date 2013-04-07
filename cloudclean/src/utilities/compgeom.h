#ifndef COMPGEOM_H
#define COMPGEOM_H

#include <Eigen/Dense>
#include "utilities/export.h"

DLLSPEC Eigen::Vector3f anyPointInPlane(Eigen::Vector4f & plane);

DLLSPEC Eigen::Vector4f pointsToPlane(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

DLLSPEC Eigen::Vector3f projPointToLine(Eigen::Vector3f p, Eigen::Vector3f a, Eigen::Vector3f b);

DLLSPEC float crossProduct2D(Eigen::Vector2f a, Eigen::Vector2f b);

DLLSPEC bool isPointInTriangle(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c);

DLLSPEC Eigen::Vector2f closestCoord(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c);

DLLSPEC float distToPlane(Eigen::Vector3f point, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

DLLSPEC float pointToLineDist(Eigen::Vector3f point,
                      Eigen::Vector3f x1,
                      Eigen::Vector3f x2);

#endif //COMPGEOM_H
