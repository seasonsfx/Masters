#ifndef MARKOV_H
#define MARKOV_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Picker;

class Markov : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "markov.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();

    ~Markov();

 private:
    void graphcut(int idx = 0);
    void randomforest();

 signals:
    void enabling();

 public slots:
    void enable();
    void disable();


 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    Picker * picker_;

    QAction * enable_;
    QAction * forrest_action_;
    bool enabled_;

    int fg_idx_;
};

#endif  // MARKOV_H
