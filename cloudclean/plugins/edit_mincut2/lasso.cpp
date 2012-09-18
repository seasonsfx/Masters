#include "lasso.h"

#include <math.h>

#include <QPolygonF>
#include <QPainter>
#include <QDebug>

#include "utilities.h"
#include <time.h>
#include <cstdlib>

Lasso::Lasso()
{
    srand ( time(NULL) );
}

int inline side(float a){
    if(a < -1e-6)
        return -1;
    if(a > 1e-6)
        return 1;
    return 0;
}

bool isPointOnLineSegment(Eigen::Vector2f lineA,
                        Eigen::Vector2f lineB,
                        Eigen::Vector2f pointC)
{
    float lineLength = (lineA - lineB).norm();
    float viaPoint = (lineA - pointC).norm() + (lineB - pointC).norm();
    return fabs(viaPoint - lineLength) < 1e-6;
}

bool oppositeSides(Eigen::Vector2f lineA, Eigen::Vector2f lineB, Eigen::Vector2f pointC, Eigen::Vector2f pointD)
{
    Eigen::Vector2f lineDir = lineA-lineB;
    Eigen::Vector2f pointDir1 = lineA-pointC;
    Eigen::Vector2f pointDir2 = lineA-pointD;

    float cross1 = lineDir.x()*pointDir1.y() - lineDir.y()*pointDir1.x();
    float cross2 = lineDir.x()*pointDir2.y() - lineDir.y()*pointDir2.x();

    int sideC = side(cross1);
    int sideD = side(cross2);
    return sideC != sideD;
}

bool intersects(Eigen::Vector2f lineA1,
                Eigen::Vector2f lineA2,
                Eigen::Vector2f lineB1,
                Eigen::Vector2f lineB2){
    if(oppositeSides(lineA1, lineA2, lineB1, lineB2) &&
            oppositeSides(lineB1, lineB2, lineA1, lineA2))
        return true;
    return false;
}

Eigen::Vector2f randomLineSegment(Eigen::Vector2f & origin){
    float rand_angle = 2.0f*M_PI*(rand() % 10000)/10000.0f;
    Eigen::Vector2f endPoint;
    endPoint << (10000.0f*cos(rand_angle) + origin.x()),
            (10000.0f*sin(rand_angle) + origin.y());
    return endPoint;
}

bool pointInsidePolygon(std::vector<Eigen::Vector2f> polygon,
                        Eigen::Vector2f point){

    while(true)
    {
        Eigen::Vector2f endPoint = randomLineSegment(point);

        for(int i = 0; i < polygon.size(); ++i)
            if(isPointOnLineSegment(point, endPoint, polygon[i]))
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

void Lasso::drawLasso(Eigen::Vector2f mouseLoc,
                      GLArea * glarea){
    QPolygonF polygon;

    // Conversion is a bit of a hack
    for(auto p: points){
        // to screen space

        polygon << screenPoint(p, glarea->width(),
                               glarea->height());
    }

    polygon << screenPoint(mouseLoc, glarea->width(),
                           glarea->height());

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
        p_2 /= p_4.z();

        /// do lasso test
        bool in_lasso = pointInsidePolygon(points, p_2);

        if(in_lasso){
            dest.push_back(idx);
        }

    }


}
