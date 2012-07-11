#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include "edit_veg.h"
#include <QIcon>
#include <QDebug>
#include <QResource>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "utilities.h"

EditVeg::EditVeg()
{

    editSample = new QAction(QIcon(":/images/veg.png"), "Vegetation select", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditVeg::~EditVeg()
{
}

void EditVeg::paintGL(CloudModel *, GLArea * glarea){

}

bool EditVeg::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    return true;
}

bool EditVeg::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){

    return true;
}
bool EditVeg::EndEdit(CloudModel *, GLArea *){
    for(auto a: actionList){
        a->setChecked(false);
    }
    return true;
}

bool EditVeg::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditVeg::mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){

    }

    return true;
}

bool EditVeg::mouseReleaseEvent(QMouseEvent *event, CloudModel *, GLArea * glarea){
    if (!glarea->moved && event->button() == Qt::LeftButton){

    }
    return true;
}

QList<QAction *> EditVeg::actions() const{
    return actionList;
}
QString EditVeg::getEditToolDescription(QAction *){
    return "Info";
}

Q_EXPORT_PLUGIN2(pnp_editveg, EditVeg)
