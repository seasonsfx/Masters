/********************************************************************************
** Form generated from reading UI file 'cloudclean.ui'
**
** Created: Wed Jun 6 14:23:36 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CLOUDCLEAN_H
#define UI_CLOUDCLEAN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QTextBrowser>
#include <QtGui/QToolBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CloudClean
{
public:
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_4;
    QPushButton *pushButton_3;
    QPushButton *pushButton;
    QGridLayout *gridLayout;
    QSlider *horizontalSlider;
    QToolBox *toolBox;
    QWidget *point_select;
    QWidget *page_2;
    QVBoxLayout *gl;
    QVBoxLayout *layers;
    QLabel *label;
    QListView *layerList;
    QHBoxLayout *buttons;
    QPushButton *openButton;
    QPushButton *pushButton_2;
    QTextBrowser *console;

    void setupUi(QWidget *CloudClean)
    {
        if (CloudClean->objectName().isEmpty())
            CloudClean->setObjectName(QString::fromUtf8("CloudClean"));
        CloudClean->resize(902, 715);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(CloudClean->sizePolicy().hasHeightForWidth());
        CloudClean->setSizePolicy(sizePolicy);
        CloudClean->setAutoFillBackground(false);
        verticalLayout_3 = new QVBoxLayout(CloudClean);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(9, -1, -1, -1);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton_4 = new QPushButton(CloudClean);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));

        verticalLayout->addWidget(pushButton_4);

        pushButton_3 = new QPushButton(CloudClean);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        verticalLayout->addWidget(pushButton_3);

        pushButton = new QPushButton(CloudClean);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalSlider = new QSlider(CloudClean);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider, 1, 0, 1, 1);

        toolBox = new QToolBox(CloudClean);
        toolBox->setObjectName(QString::fromUtf8("toolBox"));
        point_select = new QWidget();
        point_select->setObjectName(QString::fromUtf8("point_select"));
        point_select->setGeometry(QRect(0, 0, 159, 246));
        toolBox->addItem(point_select, QString::fromUtf8("Point select"));
        page_2 = new QWidget();
        page_2->setObjectName(QString::fromUtf8("page_2"));
        page_2->setGeometry(QRect(0, 0, 159, 246));
        toolBox->addItem(page_2, QString::fromUtf8("Plane select"));

        gridLayout->addWidget(toolBox, 0, 0, 1, 1);


        verticalLayout->addLayout(gridLayout);


        horizontalLayout->addLayout(verticalLayout);

        gl = new QVBoxLayout();
        gl->setSpacing(6);
        gl->setObjectName(QString::fromUtf8("gl"));

        horizontalLayout->addLayout(gl);

        layers = new QVBoxLayout();
        layers->setSpacing(6);
        layers->setObjectName(QString::fromUtf8("layers"));
        label = new QLabel(CloudClean);
        label->setObjectName(QString::fromUtf8("label"));

        layers->addWidget(label);

        layerList = new QListView(CloudClean);
        layerList->setObjectName(QString::fromUtf8("layerList"));

        layers->addWidget(layerList);


        horizontalLayout->addLayout(layers);

        horizontalLayout->setStretch(0, 3);
        horizontalLayout->setStretch(1, 10);
        horizontalLayout->setStretch(2, 3);

        verticalLayout_3->addLayout(horizontalLayout);

        buttons = new QHBoxLayout();
        buttons->setSpacing(6);
        buttons->setObjectName(QString::fromUtf8("buttons"));
        openButton = new QPushButton(CloudClean);
        openButton->setObjectName(QString::fromUtf8("openButton"));

        buttons->addWidget(openButton);

        pushButton_2 = new QPushButton(CloudClean);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        buttons->addWidget(pushButton_2);


        verticalLayout_3->addLayout(buttons);

        console = new QTextBrowser(CloudClean);
        console->setObjectName(QString::fromUtf8("console"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(console->sizePolicy().hasHeightForWidth());
        console->setSizePolicy(sizePolicy1);
        console->setMinimumSize(QSize(0, 100));

        verticalLayout_3->addWidget(console);

        verticalLayout_3->setStretch(0, 10);
        verticalLayout_3->setStretch(1, 1);

        retranslateUi(CloudClean);

        toolBox->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(CloudClean);
    } // setupUi

    void retranslateUi(QWidget *CloudClean)
    {
        CloudClean->setWindowTitle(QApplication::translate("CloudClean", "CloudClean", 0, QApplication::UnicodeUTF8));
        pushButton_4->setText(QApplication::translate("CloudClean", "Polygon Select", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("CloudClean", "Fuzzy select", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("CloudClean", "Plane select", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(point_select), QApplication::translate("CloudClean", "Point select", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(page_2), QApplication::translate("CloudClean", "Plane select", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("CloudClean", "Layers", 0, QApplication::UnicodeUTF8));
        openButton->setText(QApplication::translate("CloudClean", "Open Scan", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("CloudClean", "Save as", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CloudClean: public Ui_CloudClean {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CLOUDCLEAN_H
