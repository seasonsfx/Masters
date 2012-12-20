#ifndef TOOLBOX_H
#define TOOLBOX_H

#include <QDockWidget>
#include "cloudclean_global.h"


namespace Ui {
class Toolbox;
}

class DLLSPEC Toolbox : public QDockWidget
{
    Q_OBJECT
    
public:
    explicit Toolbox(QWidget *parent = 0);
    ~Toolbox();

public slots:
    void setSettingsWidget(QWidget * widget);
    
private:
    Ui::Toolbox *ui;
};

#endif // TOOLBOX_H
