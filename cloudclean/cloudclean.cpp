#include <QFileDialog>
#include "cloudclean.h"
#include "ui_cloudclean.h"
#include "glwidget.h"
#include "appdata.h"

CloudClean::CloudClean(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CloudClean)
{
    ui->setupUi(this);

    connect(ui->openButton, SIGNAL(pressed()), this, SLOT(loadScan()));

    if( !(AppData::Instance()->cloud->points.size() || loadScan()) ){
        exit(0);
    }

    // Add GL widget to window
    GLWidget * glwidget = new GLWidget;
    ui->gl->addWidget(glwidget);
    connect(this, SIGNAL(reloadCloud()), glwidget, SLOT(reloadCloud()) );
}

CloudClean::~CloudClean()
{
    delete ui;
}

bool CloudClean::loadScan(){
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Scan"), "~", tr("PTX Files (*.ptx)"));
	
    if(filename.length() == 0)
        return false;

    const char *ptr = filename.toAscii().data();
    AppData::Instance()->loadFile(ptr, 1);
    reloadCloud();
    return true;
}
