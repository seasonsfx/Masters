#include "settings.h"
#include "ui_settings.h"

Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);

    distanceFunc = EUCLIDIAN;
    neighbours = 4;
    threshold = 0.0f;

    connect(ui->euclid, SIGNAL(toggled(bool)), this, SLOT(changeDistFunc()));
    connect(ui->cosine, SIGNAL(toggled(bool)), this, SLOT(changeDistFunc()));
    connect(ui->thresholdSpinner, SIGNAL(valueChanged(double)), this, SLOT(changeThreshold(double)));
    connect(ui->neighboursSpinner, SIGNAL(valueChanged(int)), this, SLOT(changeNeighbours(int)));
}

Settings::~Settings()
{
    delete ui;
}

void Settings::changeDistFunc(){
    if(ui->euclid->isChecked()){
        distanceFunc = EUCLIDIAN;
    }
    if(ui->cosine->isChecked()){
        distanceFunc = COSINE;
    }
}

void Settings::changeThreshold(double val){
    threshold = val;
}

void Settings::changeNeighbours(int val){
    neighbours = val;
}
