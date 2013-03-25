#ifndef COMPGEOM_H
#define COMPGEOM_H

#include <Eigen/Dense>

Eigen::Vector3f anyPointInPlane(Eigen::Vector4f & plane);

Eigen::Vector4f pointsToPlane(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

Eigen::Vector3f projPointToLine(Eigen::Vector3f p, Eigen::Vector3f a, Eigen::Vector3f b);

float crossProduct2D(Eigen::Vector2f a, Eigen::Vector2f b);

bool isPointInTriangle(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c);

Eigen::Vector2f closestCoord(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c);

float distToPlane(Eigen::Vector3f point, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

float pointToLineDist(Eigen::Vector3f point,
                      Eigen::Vector3f x1,
                      Eigen::Vector3f x2);

#endif //COMPGEOM_H
