#include <iostream>
#include <cstdlib>
#include <assert.h>

#include <QApplication>

#include "gui/mainwindow.h"
//#include "model/cloudmodel.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    a.setApplicationName(QApplication::tr("CloudClean"));

    //CloudModel cm;
    MainWindow w;
    w.show();

    /*if (argc == 3) {
        cm.loadScan(argv[1], atoi(argv[2]));
    }

    if (argc == 2) {
        cm.loadScan(argv[1], 1);
    }
    */

    return a.exec();
}
