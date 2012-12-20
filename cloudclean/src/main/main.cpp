#include <iostream>
#include <cstdlib>
#include <assert.h>

#include <QtGui/QApplication>

#include "mainwindow.h"
#include "cloudmodel.h"

int main(int argc, char *argv[])
{ 
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    if(argc == 3){
        w.loadScan(argv[1], atoi(argv[2]));
    }

    //w.showMaximized();

    return a.exec();
}
