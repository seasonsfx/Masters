#ifndef INTERFACES_H
#define INTERFACES_H

#include <QtPlugin>
#include "cloudmodel.h"

class GLArea;

class EditPluginInterface
{
public:
    EditPluginInterface(){}
    virtual ~EditPluginInterface(){}
    virtual bool StartEdit(CloudModel &, GLArea *){return true;}
    virtual bool EndEdit(CloudModel &, GLArea *){return true;}
    virtual void mousePressEvent  (QMouseEvent *event, CloudModel &, GLArea * )=0;
    virtual void mouseMoveEvent   (QMouseEvent *event, CloudModel &, GLArea * )=0;
    virtual void mouseReleaseEvent(QMouseEvent *event, CloudModel &, GLArea * )=0;
    virtual void keyReleaseEvent  (QKeyEvent *, CloudModel &, GLArea *){}
    virtual void keyPressEvent    (QKeyEvent *, CloudModel &, GLArea *){}
    virtual void wheelEvent(QWheelEvent*, CloudModel &, GLArea * ){}
};

Q_DECLARE_INTERFACE(EditPluginInterface, "za.co.circlingthesun.cloudclean.editplugininterface/1.0")

#endif // INTERFACES_H
