#ifndef INTERFACES_H
#define INTERFACES_H

#include <QtPlugin>
#include "cloudmodel.h"
#include "glarea.h"

//class GLArea;
class QString;
class QAction;

class EditPluginInterface
{
public:
    EditPluginInterface(){}
    virtual ~EditPluginInterface(){}
    virtual bool StartEdit(CloudModel *, GLArea *){return true;}
    virtual bool EndEdit(CloudModel *, GLArea *){return true;}
    virtual void paintGL(CloudModel *, GLArea *){}
    virtual bool mouseDoubleClickEvent  (QMouseEvent *, CloudModel *, GLArea * ){return false;}
    virtual bool mousePressEvent  (QMouseEvent *, CloudModel *, GLArea * ){return false;}
    virtual bool mouseMoveEvent   (QMouseEvent *, CloudModel *, GLArea * ){return false;}
    virtual bool mouseReleaseEvent(QMouseEvent *, CloudModel *, GLArea * ){return false;}
    virtual bool keyReleaseEvent  (QKeyEvent *, CloudModel *, GLArea *){return false;}
    virtual bool keyPressEvent    (QKeyEvent *, CloudModel *, GLArea *){return false;}
    virtual bool wheelEvent(QWheelEvent*, CloudModel *, GLArea * ){return false;}
    virtual QList<QAction *> actions() const = 0;
    virtual QString getEditToolDescription(QAction *)=0;

    // Get palletr button
    // Get settings widget
};

Q_DECLARE_INTERFACE(EditPluginInterface, "za.co.circlingthesun.cloudclean.editplugininterface/1.0")

#endif // INTERFACES_H
