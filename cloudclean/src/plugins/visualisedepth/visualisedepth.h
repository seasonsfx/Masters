#ifndef VDEPTH_CUT_H
#define VDEPTH_CUT_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class QLabel;

class VDepth : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "visualisedepth.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~VDepth();

 private:

 private slots:
    void myFunc();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * myaction_ = nullptr;
    QWidget * settings_ = nullptr;
    QWidget * depth_widget_ = nullptr;
    int tab_idx_ = -1;

    QLabel * image_container = nullptr;
    QImage * image = nullptr;

};

#endif  // VDEPTH_CUT_H
