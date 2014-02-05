#ifndef JSONEXPORT_H
#define JSONEXPORT_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class JsonExport : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.jsonexport" FILE "jsonexport.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~JsonExport();

 private slots:
    void save();
    void load();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * save_action_;
    QAction * load_action_;
};

#endif  // JSONEXPORT_H
