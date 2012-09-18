#include "lasso.h"

#include <math.h>

#include <QPolygonF>
#include <QPainter>
#include <QDebug>

#include "utilities.h"

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
    Eigen::Vector2f dA = lineA - other;
    Eigen::Vector2f dB = lineB - other;
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
    Eigen::Vector2f endPoint;
    endPoint << (10000.0f*cos(angle) + origin.x()),
            (10000.0f*sin(angle) + origin.y());
    return endPoint;
}

bool pointInsidePolygon(std::vector<Eigen::Vector2f> polygon, Eigen::Vector2f point)
{
    int lastRandom = rand(1);

    while(true)
    {
        Eigen::Vector2f endPoint = randomLineSegment(point, &lastRandom);

        for(int i = 0; i < polygon.size(); ++i)
            if(pointOnLineSegment(point, endPoint, polygon[i]))
                continue;

        int hits = 0;

        for(int i = 0; i < polygon.size(); ++i){
            if(intersects(polygon[i], polygon[(i + 1) % polygon.size()], point, endPoint)){
                ++hits;
            }
        }
        return (hits % 2 == 1);
    }
}


void Lasso::addPoint(Eigen::Vector2f point){
    qDebug("New point: (%f, %f)", point.x(), point.y());
    points.push_back(point);
}

inline QPointF screenPoint(Eigen::Vector2f & p, int width, int height){
    float x = (p.x()+1)*(width/2.0f);
    float y = (-p.y()+1)*(height/2.0f);
    return QPointF(x, y);
}

void Lasso::drawLasso(Eigen::Vector2f mouseLoc, GLArea * glarea){
    QPolygonF polygon;

    // Conversion is a bit of a hack
    for(auto p: points){
        // to screen space

        polygon << screenPoint(p, glarea->width(), glarea->height());
    }

    polygon << screenPoint(mouseLoc, glarea->width(), glarea->height());

    qDebug("Drawing polygon");
    qDebug() << polygon;

    QPainter painter(glarea);
    painter.beginNativePainting();
    painter.setPen(Qt::green);
    painter.drawPolygon(polygon);
    painter.endNativePainting();

    glError("Paint polygon failed");
}

void Lasso::clear(){
    points.clear();
}

std::vector<Eigen::Vector2f> Lasso::getPolygon(){
    return points;
}

void Lasso::getIndices(Eigen::Matrix4f & ndc_mat,
                pcl::PointCloud<pcl::PointXYZI> * cloud,
                std::vector<int> & source,
                std::vector<int> & dest){


    float * matdata = ndc_mat.data();

    for(int idx : source){

        if(idx == -1)
            continue;

        pcl::PointXYZI & p = cloud->points[idx];

        /// project point
        Eigen::Vector4f p_4;
        p_4 << p.x, p.y, p.z, 1;
        p_4 = ndc_mat * p_4;

        Eigen::Vector2f p_2;
        p_2 << p_4.x(), p_4.y();

        /// do lasso test
        bool in_lasso = pointInsidePolygon(points, p_2);

        if(in_lasso){
            dest.push_back(idx);
        }

    }


}
