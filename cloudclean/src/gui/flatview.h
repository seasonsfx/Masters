#ifndef FLATVIEW_H
#define FLATVIEW_H

#include "glheaders.h"
#include <QGLWidget>
#include <QImage>
#include <QPainter>
#include <QVector2D>
#include <QGLShaderProgram>
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "model/pointcloud.h"
#include "gui/gldata.h"
#include <Eigen/Dense>

class FlatView : public QGLWidget {
    Q_OBJECT
 public:
    FlatView(QGLFormat &fmt, CloudList * cl,
             LayerList * ll, QWidget *parent = 0,
             QGLWidget * sharing = 0);
    void setGLD(GLData *gld);
    const Eigen::Matrix3f getCamera();

    int imageToCloudIdx(int x, int y);
    QPoint cloudToImageCoord(int idx);

 public slots:
    void setCloud(std::shared_ptr<PointCloud> new_pc);

 private slots:
  void contextMenu(const QPoint &pos);

 signals:
  void flagUpdate();
  void labelUpdate();

 protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    void mouseMoveEvent(QMouseEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);
    void wheelEvent(QWheelEvent * event);

 private:
   std::vector<int> cloud_idx_lookup_;
   std::weak_ptr<PointCloud> pc_;
   CloudList * cl_;
   LayerList * ll_;
   GLData * gld_;

   QGLShaderProgram program_;

   int uni_sampler_;
   int uni_select_color_;
   int uni_width_;
   int uni_height_;
   int uni_camera_;
   GLuint texture_id_;
   GLuint vao_;

   float current_scale_;
   QVector2D saved_offset_;
   QVector2D aspect_ratio_;

   QPoint drag_start_pos;
   Eigen::Matrix3f camera_;

};

#endif // FLATVIEW_H
