#include "lasso.h"

#include <GL/glu.h>
#include <math.h>

#include <QPolygonF>
#include <QPainter>
#include <QDebug>
#include <QPen>

#include <time.h>
#include <cstdlib>

Lasso::Lasso()
{

}

Eigen::Vector2i Lasso::getScreenPoint(Eigen::Vector2f & p, int w, int h) {
    float x = (p.x()+1)*(w/2.0f);
    float y = (-p.y()+1)*(h/2.0f);
    return Eigen::Vector2i(x, y);
}

Eigen::Vector2f Lasso::NDCPoint(Eigen::Vector2i & p, int w, int h) {
    float fx = 2.0*float(p.x())/w-1.0f;
    float fy = -2.0*float(p.y())/h+1.0f;
    return Eigen::Vector2f(fx, fy);
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

    while(true) {
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




void Lasso::addNormPoint(Eigen::Vector2f point) {
    points_.push_back(point);
}

void Lasso::moveLastScreenPoint(int x, int y, QPaintDevice *device) {
    float fx = 2.0*float(x)/device->width()-1.0f;
    float fy = -2.0*float(y)/device->height()+1.0f;
    if(points_.size() != 0)
        points_.pop_back();
    addNormPoint(Eigen::Vector2f(fx, fy));
}

void Lasso::addScreenPoint(int x, int y, int w, int h) {
    float fx = 2.0*float(x)/w-1.0f;
    float fy = -2.0*float(y)/h+1.0f;
    addNormPoint(Eigen::Vector2f(fx, fy));
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
    for(auto p: points_){
        polygon << screenPoint(p, device->width(),
                               device->height());
    }

    polygon << screenPoint(mouseLoc, device->width(),
                           device->height());


    QPainter painter(device);
    painter.endNativePainting();
    painter.setPen(Qt::green);
    painter.drawPolygon(polygon); CE();

    QPen pen(Qt::red);
    pen.setCapStyle(Qt::RoundCap);
    pen.setWidth(6);

    painter.setPen(pen);
    painter.drawPoints(polygon);

    painter.beginNativePainting();

}

void Lasso::clear(){
    points_.clear();
}

std::vector<Eigen::Vector2f> Lasso::getPolygon(){
    return points_;
}

std::vector<Eigen::Vector2f> Lasso::getPoints() {
    return points_;
}

void Lasso::getIndices(Eigen::Affine3f & proj,
                        Eigen::Affine3f & mv,
                pcl::PointCloud<pcl::PointXYZI> * cloud,
                boost::shared_ptr<std::vector<int> > source_indices){

    Eigen::Affine3f ndc_mat = proj * mv;

    int count = 10;

    auto copyd = [] (float * i, double * o) {
        for(int idx = 0; idx < 16; idx++){
            o[idx] = i[idx];
        }
    };

    double mv1 [16];
    double proj1 [16];
    copyd(proj.data(), proj1);
    copyd(mv.data(), mv1);

    int view [4] = {0, 0, 1000, 1000};

    auto inside_lasso = [&] (pcl::PointXYZI & p) {
        /// project point
        Eigen::Vector4f p_4 = p.getVector4fMap();
        p_4 = ndc_mat.matrix() * p_4;

        // Limit to front of camera
        if(p_4.z() < 0.0f)
            return false;

        // Perspective divide
        //Eigen::Vector2f p_2;
        //p_2 << p_4.x(), p_4.y();
        //p_2 /= p_4.w();

        double wx, wy, wz;
        gluProject(p.x, p.y, p.z, mv1, proj1, view, &wx, &wy, &wz);
        wx = (wx/500.0f)-1.0f, wy = (wy/500.0f)-1.0f;

        Eigen::Vector2f p_2(wx, wy);

        if(count-- > 0) {
            qDebug() << p_4.w();
            std::cout << ndc_mat.matrix() << std::endl;
        }


        /// do lasso test
        return  pointInsidePolygon(points_, p_2);
    };

    if(source_indices->size() == 0) {
        for (uint idx = 0; idx < cloud->size(); idx++) {
            if(inside_lasso(cloud->points[idx])) {
                source_indices->push_back(idx);
            }
        }
    }

}

void Lasso::getIndices2D(int height, const Eigen::Affine2f & cam,
                const std::vector<int> & cloud_to_grid_map,
                boost::shared_ptr<std::vector<int> > source_indices) {

    auto inside_lasso = [&] (int idx) {
        /// do lasso test
        int i = cloud_to_grid_map[idx];

        Eigen::Vector2f point = cam * Eigen::Vector2f(i/height, i%height);

        return  pointInsidePolygon(points_, point);
    };

    if(source_indices->size() == 0) {
        for (uint idx = 0; idx < cloud_to_grid_map.size(); idx++) {
            if(inside_lasso(idx)) {
                source_indices->push_back(idx);
            }
        }
    }

}
