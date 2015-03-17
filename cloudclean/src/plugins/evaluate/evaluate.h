#ifndef EVALUATE_H
#define EVALUATE_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class Evaluate : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.evaluate" FILE "evaluate.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Evaluate();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void filter();

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

    float radius_;
};

#endif  // EVALUATE_H
