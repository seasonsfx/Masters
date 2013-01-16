#include <iostream>
#include <cstdlib>
#include <assert.h>

#include <QtGui/QApplication>

#include "mainwindow.h"
#include "cloudmodel.h"

int main(int argc, char *argv[])
{ 
    QApplication a(argc, argv);
    a.setApplicationName(QApplication::tr("CloudClean"));

    MainWindow w;
    w.show();

    if(argc == 3){
        w.loadScan(argv[1], atoi(argv[2]));
    }

    if(argc == 2){
        w.loadScan(argv[1], 1);
    }

    //w.showMaximized();

    return a.exec();
}
