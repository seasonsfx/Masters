#define GL3_PROTOTYPES
#include <gl3.h>
#include "edit_sample.h"
#include <QIcon>
#include <QDebug>
#include <QResource>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "utilities.h"

EditSample::EditSample()
{

    editSample = new QAction(QIcon(":/images/lasso.png"), "Sample edit", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditSample::~EditSample()
{
}

void EditSample::paintGL(CloudModel *, GLArea * glarea){

}

bool EditSample::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    return true;
}

bool EditSample::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){

    return true;
}
bool EditSample::EndEdit(CloudModel *, GLArea *){
    for(auto a: actionList){
        a->setChecked(false);
    }
    return true;
}

bool EditSample::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditSample::mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){

    }

    return true;
}

bool EditSample::mouseReleaseEvent(QMouseEvent *event, CloudModel *, GLArea * glarea){
    if (!glarea->moved && event->button() == Qt::LeftButton){

    }
    return true;
}

QList<QAction *> EditSample::actions() const{
    return actionList;
}
QString EditSample::getEditToolDescription(QAction *){
    return "Info";
}

Q_EXPORT_PLUGIN2(pnp_editsample, EditSample)
