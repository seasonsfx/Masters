#ifndef SETTINGS_H
#define SETTINGS_H

#include <QWidget>

namespace Ui {
class Settings;
}

enum DistanceEnum{EUCLIDIAN, COSINE};

class Settings : public QWidget
{
    Q_OBJECT
    
public:
    explicit Settings(QWidget *parent = 0);
    ~Settings();

public slots:
    void changeDistFunc();
    void changeThreshold(double val);
    void changeNeighbours(int val);
    
private:
    Ui::Settings *ui;

public:
    DistanceEnum distanceFunc;
    float threshold;
    int neighbours;
};

#endif // SETTINGS_H
