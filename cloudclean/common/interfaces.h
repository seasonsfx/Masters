#ifndef INTERFACES_H
#define INTERFACES_H

#include <QtPlugin>

//#include "cloudmodel.h"
//#include "glarea.h"

class GLArea;
class CloudModel;
class QMouseEvent;
class QKeyEvent;
class QWheelEvent;
class QString;
class QAction;

class EditPluginInterface
{
public:
    EditPluginInterface(){}
    virtual ~EditPluginInterface(){}
    virtual bool StartEdit(QAction *, CloudModel *, GLArea *){return true;}
    virtual bool EndEdit(CloudModel *, GLArea *){return true;}
    virtual void paintGL(CloudModel *, GLArea *){}
    virtual bool mouseDoubleClickEvent  (QMouseEvent *, CloudModel *, GLArea * ){return true;}
    virtual bool mousePressEvent  (QMouseEvent *, CloudModel *, GLArea * ){return true;}
    virtual bool mouseMoveEvent   (QMouseEvent *, CloudModel *, GLArea * ){return true;}
    virtual bool mouseReleaseEvent(QMouseEvent *, CloudModel *, GLArea * ){return true;}
    virtual bool keyReleaseEvent  (QKeyEvent *, CloudModel *, GLArea *){return true;}
    virtual bool keyPressEvent    (QKeyEvent *, CloudModel *, GLArea *){return true;}
    virtual bool wheelEvent(QWheelEvent*, CloudModel *, GLArea * ){return true;}
    virtual QList<QAction *> actions() const = 0;
    virtual QString getEditToolDescription(QAction *)=0;
    virtual QWidget * getSettingsWidget(QWidget *){return NULL;}

    // Get palletr button
    // Get settings widget
};

Q_DECLARE_INTERFACE(EditPluginInterface, "za.co.circlingthesun.cloudclean.editplugininterface/1.0")

class VizPluginInterface
{
public:
    VizPluginInterface(){}
    virtual ~VizPluginInterface(){}
    virtual bool StartViz(QAction *, CloudModel *, GLArea *){return true;}
    virtual bool EndViz(CloudModel *, GLArea *){return true;}
    virtual void paintLayer(int, CloudModel *, GLArea *){}
    virtual void paintGL(CloudModel *, GLArea *){}
    virtual QList<QAction *> actions() const = 0;
    virtual QString getEditToolDescription(QAction *)=0;
    virtual QWidget * getSettingsWidget(QWidget *){return NULL;}
};

Q_DECLARE_INTERFACE(VizPluginInterface, "za.co.circlingthesun.cloudclean.vizplugininterface/1.0")

#endif // INTERFACES_H
