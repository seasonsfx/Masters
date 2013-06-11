#ifndef VDEPTH_CUT_H
#define VDEPTH_CUT_H

#include "glheaders.h"
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

    QAction * myaction_;
    QWidget * settings_;
    QWidget * depth_widget_;
    int tab_idx_;

    QLabel * image_container_;
    QImage * image_;

};

#endif  // VDEPTH_CUT_H
