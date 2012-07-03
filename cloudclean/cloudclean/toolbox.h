#ifndef TOOLBOX_H
#define TOOLBOX_H

#include <QDockWidget>

namespace Ui {
class Toolbox;
}

class Toolbox : public QDockWidget
{
    Q_OBJECT
    
public:
    explicit Toolbox(QWidget *parent = 0);
    ~Toolbox();
    
private:
    Ui::Toolbox *ui;
};

#endif // TOOLBOX_H
