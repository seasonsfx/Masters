#ifndef ACCURACY_H
#define ACCURACY_H

#include <set>
#include <boost/weak_ptr.hpp>
#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Layer;
class QLineEdit;
class QLabel;
class QDoubleSpinBox;

class Accuracy : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.accuracy" FILE "accuracy.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Accuracy();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void accuracy();

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

    float target_accuracy_;

    //QLineEdit * precision_;
    //QLineEdit * recall_;
    QDoubleSpinBox * target_accuracy_input_;
    QAction * fscore_action_;

    std::vector<boost::weak_ptr<Layer> >  target_;
};

#endif  // ACCURACY_H
