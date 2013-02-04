#include "gui/mainwindow.h"
//#include <QtWidgets>
#include "gui/glwidget.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent) {
		glWidget = new GLWidget;
		QGLFormat base_format = glWidget->format();
		base_format.setVersion(3, 3);
		base_format.setProfile(QGLFormat::CoreProfile);
		glWidget->setFormat(base_format);
        this->setCentralWidget(glWidget);
}

MainWindow::~MainWindow() {

}
