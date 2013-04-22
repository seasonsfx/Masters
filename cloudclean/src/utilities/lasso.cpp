#include "lasso.h"

#include <math.h>

#include <QPolygonF>
#include <QPainter>
#include <QDebug>

#include <time.h>
#include <cstdlib>

Lasso::Lasso()
{

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

bool oppositeSides(Eigen::Vector2f lineStart, Eigen::Vector2f lineEnd, Eigen::Vector2f pointA, Eigen::Vector2f pointB)
{
    Eigen::Vector2f lineDir = lineStart-lineEnd;
    Eigen::Vector2f pointDir1 = lineStart-pointA;
    Eigen::Vector2f pointDir2 = lineStart-pointB;

    float cross1 = lineDir.x()*pointDir1.y() - lineDir.y()*pointDir1.x();
    float cross2 = lineDir.x()*pointDir2.y() - lineDir.y()*pointDir2.x();

    int sideC = side(cross1);
    int sideD = side(cross2);
    return sideC != sideD;
}

bool intersects(Eigen::Vector2f line1Start,
                Eigen::Vector2f line1End,
                Eigen::Vector2f line2Start,
                Eigen::Vector2f line2End) {
    if(oppositeSides(line1Start, line1End, line2Start, line2End) &&
            oppositeSides(line2Start, line2End, line1Start, line1End))
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

        for(uint i = 0; i < polygon.size(); ++i)
            if(isPointOnLineSegment(point, endPoint, polygon[i]))
                continue;

        int hits = 0;

        for(uint i = 0; i < polygon.size(); ++i){
            if(intersects(polygon[i], polygon[(i + 1) % polygon.size()], point, endPoint)){
                ++hits;
            }
        }
        return (hits % 2 == 1);
    }
}


void Lasso::addPoint(Eigen::Vector2f point) {
    points.push_back(point);
}

void Lasso::movePoint(int x, int y, QPaintDevice *device) {
    float fx = 2.0*float(x)/device->width()-1.0f;
    float fy = -2.0*float(y)/device->height()+1.0f;
    if(points.size() != 0)
        points.pop_back();
    addPoint(Eigen::Vector2f(fx, fy));
}

void Lasso::addPoint(int x, int y, QPaintDevice *device) {
    float fx = 2.0*float(x)/device->width()-1.0f;
    float fy = -2.0*float(y)/device->height()+1.0f;
    addPoint(Eigen::Vector2f(fx, fy));
}

inline QPointF screenPoint(Eigen::Vector2f & p, int width, int height){
    float x = (p.x()+1)*(width/2.0f);
    float y = (-p.y()+1)*(height/2.0f);
    return QPointF(x, y);
}

void Lasso::drawLasso(int x, int y, QPaintDevice *device) {
    float fx = 2.0*float(x)/device->width()-1.0f;
    float fy = -2.0*float(y)/device->height()+1.0f;
    drawLasso(Eigen::Vector2f(fx, fy), device);
}

void Lasso::drawLasso(Eigen::Vector2f mouseLoc, QPaintDevice * device){
    QPolygonF polygon;

    // Conversion is a bit of a hack
    for(auto p: points){
        polygon << screenPoint(p, device->width(),
                               device->height());
    }

    polygon << screenPoint(mouseLoc, device->width(),
                           device->height());


    QPainter painter(device);
    painter.endNativePainting();
    painter.setPen(Qt::green);
    painter.drawPolygon(polygon); CE();
    painter.beginNativePainting();

}

void Lasso::clear(){
    points.clear();
}

std::vector<Eigen::Vector2f> Lasso::getPolygon(){
    return points;
}

void Lasso::getIndices(Eigen::Matrix4f & ndc_mat,
                pcl::PointCloud<pcl::PointXYZI> * cloud,
                std::shared_ptr<std::vector<int> > source_indices,
                std::shared_ptr<std::vector<int> > removed_indices){

    float * matdata = ndc_mat.data();

    auto inside_lasso = [&, matdata] (pcl::PointXYZI & p) {
        /// project point
        Eigen::Vector4f p_4;
        p_4 << p.x, p.y, p.z, 1;
        p_4 = ndc_mat * p_4;

        Eigen::Vector2f p_2;
        p_2 << p_4.x(), p_4.y();
        p_2 /= p_4.z();

        /// do lasso test
        return  pointInsidePolygon(points, p_2);
    };

    if(source_indices->size() == 0) {
        for (int idx = 0; idx < cloud->size(); idx++) {
            if(inside_lasso(cloud->points[idx])) {
                source_indices->push_back(idx);
            }
        }
    } else {
        for(int idx : *source_indices){
            if(!inside_lasso(cloud->points[idx])) {
                removed_indices->push_back(idx);
            }
        }
    }

}
