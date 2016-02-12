#ifndef CAMERASETTER_H
#define CAMERASETTER_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

#include <Eigen/Core>
#include <QTimer>
#include <QTime>

class CameraSetter : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.camerasetter" FILE "camerasetter.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~CameraSetter();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void set();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QWidget * settings_;
    bool is_enabled_;

    Eigen::Vector3f target_;
    bool running_;
    bool ready_to_start_ = true;
    int seconds_;
    QTimer timer_;
    QTime time_;
};

#endif  // CAMERASETTER_H
