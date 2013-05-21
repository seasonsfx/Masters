#ifndef GRAPH_CUT_H
#define GRAPH_CUT_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class Select;
class QMouseEvent;

#include <Eigen/Dense>

#include "glheaders.h"


class Stub : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "stub.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Stub();

    bool eventFilter(QObject *object, QEvent *event);

 private slots:
    void myFunc();

////

signals:
   void enabling();

public slots:
   void enable();
   void disable();

private slots:
   void completeSegment(Select * select);

private:
   void segment(int idx);
   bool mouseClickEvent(QMouseEvent * event);
   bool mouseMoveEvent(QMouseEvent * event);
   bool mousePressEvent(QMouseEvent * event);
   bool mouseReleaseEvent(QMouseEvent * event);

////


 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;


    float radius_;
    int k_connect_;
    float source_weight_;
    float sigma_;

    QAction * myaction;
    QWidget * settings_;


    QAction * enable_;

    Eigen::Vector2d last_mouse_pos_;
    Eigen::Vector2d mouse_down_pos_;

    bool is_enabled_;


};

#endif  // GRAPH_CUT_H
