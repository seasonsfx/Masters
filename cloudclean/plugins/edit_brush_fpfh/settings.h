#ifndef SETTINGS_H
#define SETTINGS_H

#include <QWidget>

namespace Ui {
class Settings;
}

enum DistanceEnum{EUCLIDIAN, COSINE, INTENSITY};
enum NeigbourEnum{K, FIXEDRAD, DYNRAD};

class Settings : public QWidget
{
    Q_OBJECT
    
public:
    explicit Settings(QWidget *parent = 0);
    ~Settings();

    int kNeigbours() const;
    float fixedRad() const;
    float dynRad() const;

    int euclidianDist() const;
    float cosineDist() const;
    float intensityDist() const;

public slots:
    void changeDistFunc();
    void changeNeigbourFunc();
    
private:
    Ui::Settings *ui;

public:
    DistanceEnum distanceFunc;
    NeigbourEnum neighbourFunc;
    float dynradius;
    float fixedradius;
    int k;
};

#endif // SETTINGS_H
