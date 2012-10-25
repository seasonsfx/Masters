#include "settings.h"
#include "ui_settings.h"

Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);
    connect(ui->showGraph, SIGNAL(clicked()), this, SLOT(graphToggle()));
    connect(ui->edgeWidth, SIGNAL(valueChanged(double)), this, SLOT(graphToggle()));
}

Settings::~Settings()
{
    delete ui;
}

int Settings::kConnectvity(){
    return ui->kNeigbours->value();
}

float Settings::sigma(){
    return ui->sigma->value();
}

float Settings::radius(){
    return ui->radius->value();
}

bool Settings::showGraph(){
    return ui->showGraph->isChecked();
}

float Settings::sourceWeight(){
    return ui->sourceWeight->value();
}

void Settings::graphToggle(){
    emit repaint();
}

float Settings::vertexSize(){
    return ui->vextexSize->value();
}

float Settings::edgeWidth(){
    return ui->edgeWidth->value();
}
