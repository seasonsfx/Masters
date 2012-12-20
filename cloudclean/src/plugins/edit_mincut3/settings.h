#ifndef SETTINGS_H
#define SETTINGS_H

#include <QWidget>
#include <Eigen/Dense>

namespace Ui {
class Settings;
}

class Settings : public QWidget
{
    Q_OBJECT
    
public:
    explicit Settings(QWidget *parent = 0);
    ~Settings();
    
    Eigen::Vector3f ratio();
    float deviation();

    int kConnectvity();
    float sigma();
    float radius();
    bool showGraph();
    float sourceWeight();

private:
    Ui::Settings *ui;
};

#endif // SETTINGS_H
