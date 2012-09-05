#include "settings.h"
#include "ui_settings.h"

Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);
}

Settings::~Settings()
{
    delete ui;
}


Eigen::Vector3f Settings::ratio(){
    float eigenvalues[3] = {(float) ui->eigen1->value(),
                            (float) ui->eigen2->value(),
                            (float) ui->eigen3->value()};

    for(int j = 0; j < 3; j++ ){
        int max = j;
        for(int k = j; k < 3; k++ ){
            if(eigenvalues[k] > eigenvalues[max])
                max = k;
        }
        float tmp = eigenvalues[j];
        eigenvalues[j] = eigenvalues[max];
        eigenvalues[max] = tmp;
    }

    return Eigen::Vector3f(eigenvalues[0],
                           eigenvalues[1],
                           eigenvalues[2]).normalized();

}

float Settings::deviation(){
    return ui->deviation->value();
}

int Settings::kConnectvity(){
    return ui->kConnect->value();
}

int Settings::kPCA(){
    return ui->kPCA->value();
}
