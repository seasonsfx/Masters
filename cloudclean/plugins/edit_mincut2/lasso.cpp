#include "lasso.h"

#include <math.h>

#include <QPolygonF>
#include <QPainter>

Lasso::Lasso()
{
}

int rand(int value)
{
    const int a = 1103515245;
    const int c = 12345;

    return (a*value) + c;
}

float cross2D(Eigen::Vector2f lineA, Eigen::Vector2f lineB, Eigen::Vector2f other)
{
    Eigen::Vector2f dA, dB;
    dA.x() = lineA.x() - other.x(); dA.y() = lineA.y() - other.y();
    dB.x() = lineB.x() - other.x(); dB.y() = lineB.y() - other.y();

    return dA.x()*dB.y() - dA.y()*dB.x();
}

int side(float a)
{
    if(a < -1e-6)
        return -1;
    if(a > 1e-6)
        return 1;
    return 0;
}


inline float pointDistance(Eigen::Vector2f & pointA, Eigen::Vector2f & pointB)
{
    return (pointA - pointB).norm();
}

bool pointOnLineSegment(Eigen::Vector2f lineA, Eigen::Vector2f lineB, Eigen::Vector2f pointC)
{
    float lineLength = pointDistance(lineA, lineB);
    float viaPoint = pointDistance(lineA, pointC) + pointDistance(lineB, pointC);
    return fabs(viaPoint - lineLength) < 1e-6;
}

bool oppositeSides(Eigen::Vector2f lineA, Eigen::Vector2f lineB, Eigen::Vector2f pointC, Eigen::Vector2f pointD)
{
    float crossC = cross2D(lineA, lineB, pointC);
    float crossD = cross2D(lineA, lineB, pointD);

    int sideC = side(crossC);
    int sideD = side(crossD);

    return sideC != sideD;
}

bool intersects(Eigen::Vector2f lineA1, Eigen::Vector2f lineA2, Eigen::Vector2f lineB1, Eigen::Vector2f lineB2)
{
    if(oppositeSides(lineA1, lineA2, lineB1, lineB2) && oppositeSides(lineB1, lineB2, lineA1, lineA2))
        return true;
    return false;
}

float randomAngle(int* lastRandom)
{
    *lastRandom = rand(*lastRandom);
    return 2.0f*M_PI*(*lastRandom % 10000)/10000.0f;
}

Eigen::Vector2f randomLineSegment(Eigen::Vector2f & origin, int* lastRandom)
{
    float angle = randomAngle(lastRandom);
    Eigen::Vector2f endPoint();
    endPoint << 10000.0f*cos(angle) + origin.x(),
            10000.0f*sin(angle) + origin.y();
    return endPoint;
}

bool pointInsidePolygon(std::vector<Eigen::Vector2f> polygon, Eigen::Vector2f point)
{
    int lastRandom = rand(1);

    while(true)
    {
        Eigen::Vector2f endPoint = randomLineSegment(point, &lastRandom);

        for(int i = 0; i < n; ++i)
            if(pointOnLineSegment(point, endPoint, polygon[i]))
                continue;

        int hits = 0;

        for(int i = 0; i < n; ++i){
            if(intersects(polygon[i], polygon[(i + 1) % n], point, endPoint)){
                ++hits;
            }
        }
        return (hits % 2 == 1);
    }
}


void Lasso::addPoint(Eigen::Vector2f point){
    points.push_back(point);
}

void Lasso::drawLasso(Eigen::Vector2f mouseLoc, GLArea * glarea){
    QPolygonF polygon;

    // Conversion is a bit of a hack
    for(auto p: points){
        polygon << QPointF(p.x(), p.y());
    }

    polygon << QPointF(mouseLoc.x(), mouseLoc.y());

    QPainter painter(glarea);
    painter.beginNativePainting();
    painter.setPen(Qt::green);
    painter.drawPolygon(polygon);
    painter.endNativePainting();
}

void Lasso::clear(){
    points.clear();
}

std::vector<Eigen::Vector2f> Lasso::getPolygon(){
    return points;
}

void Lasso::getIndices(Eigen::Matrix4f & ndc_mat,
                pcl::PointCloud<pcl::PointXYZI> & cloud,
                std::vector<int> & source,
                std::vector<int> & dest){


    float * matdata = ndc_mat.data();
    dest.resize(source.size());

    for(int i = 0; i < source.size(); i++){
        int idx = i;

        if(idx == -1)
            continue;

        pcl::PointXYZI & p = cm->cloud->points[idx];
        float point[4] = {p.x, p.y, p.z, p.intensity};

        /// project point
        proj(matdata, point);

        /// do lasso test
        bool in_lasso = pointInsidePolygon(points, point);

        if(in_lasso){
            dest[idx] = source_indices[idx];
        }

    }


}
