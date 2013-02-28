#ifndef CLOUDGLDATA_H
#define CLOUDGLDATA_H

#include "glheaders.h"
#include "model/pointcloud.h"
#include <QObject>
#include <QGLBuffer>

class CloudGLData : public QObject{
    Q_OBJECT
 public:
    CloudGLData(std::shared_ptr<PointCloud> pc);
    ~CloudGLData();

    void setVAO(GLuint vao);

    void draw(GLint vao);

    void copyCloud();
    void copyLabels();
    void copyFlags();
    void copyGrid();
    // TOD0: enableGrid()

 public slots:
    void syncCloud();
    void syncLabels();
    void syncFlags();

 public:
    std::shared_ptr<PointCloud> pc_;
    std::shared_ptr<QGLBuffer> label_buffer_;
    std::shared_ptr<QGLBuffer> point_buffer_;
    std::shared_ptr<QGLBuffer> flag_buffer_;
    std::shared_ptr<QGLBuffer> grid_buffer_;

 private:
    bool dirty_labels_;
    bool dirty_points_;
    bool dirty_flags_;
    bool dirty_grid_;
};


#endif // CLOUDGLDATA_H
